/*
   Copyright 2018 Simon Vogl <svogl@voxel.at>
                  Angel Merino-Sastre <amerino@voxel.at>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include <iostream>
#include <stdexcept>
#include <system_error>
#include <thread>


#include <toffy/controller.hpp>
#include <toffy/parallelFilter.hpp>
#include <toffy/common/plugins.hpp>

#include <opencv2/highgui.hpp>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

// POSIX only: toffy has no Windows plugin loader (P2-2, cleanup/findings/build.md).
#include <dlfcn.h>

using namespace toffy;
using namespace std;

/*Controller * Controller::_controller;


Controller * Controller::getInstance() {
    if(_controller == NULL) {
	_controller = new Controller();
    }
    return _controller;
}*/

Controller::Controller() : baseFilterBank(NULL), _state(Controller::IDLE)
{
    toffy::log::debug() << __FUNCTION__;
    baseFilterBank = std::static_pointer_cast<FilterBank>(
        toffy::FilterFactory::getInstance()->createFilter("filterBank",
                                                          "baseController"));
    // createFilter() returns null for a type it cannot resolve, and ->bank(NULL)
    // was then an unconditional null dereference. Fail loudly instead: a
    // Controller without its base bank can never run a pipeline, so constructing
    // one only moves the failure to an obscure later point. Throwing also means
    // ~Controller never runs on a half-built object.
    if (!baseFilterBank)
        throw std::runtime_error(
            "Controller: FilterFactory could not create the base FilterBank");
    baseFilterBank->bank(NULL);
}

Controller::~Controller()
{
    toffy::log::debug() << __FUNCTION__;
    _state = Controller::IDLE;
    // Releasing the root bank is now just letting the FilterPtr go (P2-5): the
    // bank and everything pipelined into it are destroyed here, by this object,
    // which is what has always owned them. The line that used to do this was
    //
    //     FilterFactory::getInstance()->deleteFilter(baseFilterBank->id());
    //
    // i.e. a request to a process-wide registry to `delete` an object owned
    // per-Controller. It destroyed the bank through a name, so it also
    // invalidated every other pointer to it -- and it was the reason
    // ~FilterBank::clearBank() had to race the factory on the way out.
    //
    // reset() is explicit rather than leaving the member to destruct after this
    // body, because any teardown step below wants the pipeline to be gone
    // *before* it runs: ~FilterBank can call into filters (it calls their
    // destructors), and those run while the factory is still intact only if the
    // bank dies first.
    baseFilterBank.reset();
    // The `clearCreators()` call that used to sit here is gone.
    //
    // It wiped the *process-wide* creator registry from the destructor of one
    // object, so destroying a Controller threw away every plug-in and
    // Player::loadFilter() creator in the process (finding C3). It survived
    // review for as long as it did because it was invisible from the built-in
    // types: createFilter() resolved those through a hard-coded chain, so a
    // second Controller still worked and only a consumer's own filters went --
    // and no test registered one. `DestroyingAControllerKeepsRegisteredCreators`
    // does, and fails with the call in place.
    //
    // P2-5 is what made this call removable at all: once the bank is released by
    // letting the FilterPtr go, nothing in the factory needs emptying to keep
    // teardown safe. The behaviour change is the correct one -- a second
    // Controller now keeps the plug-ins the first registered, which is what
    // `boxes` and `toffy-oatpp` already assume, both registering their filters
    // once at startup (`init_toffy()`) and naming them from config afterwards.
    //
    // The method itself stays: it is public API, and removing it is an X6 break.
    // Nothing in toffy calls it any more.
    // P2-10, closing the TODO that was here: the plug-in handles are no longer
    // this object's to drain. They have one owner, toffy::commons::plugins, and
    // they are deliberately never dlclose()d -- a plug-in's init() registers
    // creators into the process-wide FilterFactory, and those creators are
    // function pointers into the .so, so unloading it while the registry still
    // holds them makes the next createFilter() jump into unmapped memory.
    // Reinstating unload needs the registry to give creators back first; that is
    // a FilterFactory change, not a teardown detail here, and it is why this is
    // documented rather than implemented. Controller never owned them correctly
    // in the first place: FilterBank's copy of the loop discarded them outright.
}

namespace {
// Every parallelFilter in the bank, in pipeline order. All four run methods
// needed this; P2-6 collapses them into one, so it is here once.
std::vector<Filter *> parallelFiltersIn(toffy::FilterBank *bank)
{
    std::vector<Filter *> vec;
    bank->getFiltersByType("parallelFilter", vec);
    return vec;
}
}  // namespace

bool Controller::run(Controller::state newState, bool singleStep)
{
    if (_state != Controller::IDLE) {
        // Already running. A second launch would assign over a joinable
        // std::thread, which calls std::terminate().
        return false;
    }

    std::vector<Filter *> vec = parallelFiltersIn(baseFilterBank.get());
    for (size_t i = 0; i < vec.size(); i++) {
        static_cast<ParallelFilter *>(vec[i])->init();
        static_cast<ParallelFilter *>(vec[i])->start();
    }

    if (singleStep) {
        // A single step runs the bank on the calling thread and deliberately
        // never touches _state -- which is what lets a step follow another step,
        // and what the collapse must not "tidy" away.
        const bool backwards = (newState == Controller::BACKWARD);
        if (backwards) f.addData("backward", true);
        baseFilterBank->filter(f, f);
        if (backwards) f.removeData("backward");
        for (size_t i = 0; i < vec.size(); i++) {
            static_cast<ParallelFilter *>(vec[i])->stop();
        }
        return true;
    }

    _state = newState;
    // Collect a previous run before starting a new one: loopFilters() puts the
    // state back to IDLE itself when the bank is empty, so arriving here with a
    // finished but still joinable thread is normal. Assigning over it calls
    // std::terminate(); boost::thread at BOOST_THREAD_VERSION 2 turned the same
    // assignment into a detached worker nobody owned.
    if (_thread.joinable()) _thread.join();
    try {
        _thread = std::thread(&Controller::loopFilters, this);
    } catch (const std::system_error &e) {
        // std::thread reports a failed launch by throwing. The
        // `if (!_thread.joinable())` check that used to follow the assignment
        // could never fire: boost::thread threw as well.
        _state = Controller::IDLE;
        toffy::log::error() << "Could not start the filter thread: " << e.what();
        return false;
    }
    return true;
}

bool Controller::forward() { return run(Controller::FORWARD, false); }

bool Controller::backward() { return run(Controller::BACKWARD, false); }

bool Controller::stepForward() { return run(Controller::FORWARD, true); }

bool Controller::stepBackward() { return run(Controller::BACKWARD, true); }

bool Controller::stop()
{
    _state = Controller::IDLE;
    // Join only a thread that was actually started. _thread becomes joinable in
    // forward()/backward()/stepForward()/stepBackward(); joining it unconditionally
    // made stop() throw whenever it ran before any of them (finding A12). The rest
    // of A12 -- the four duplicated run methods -- is left to P2-6. The
    // assignments P2-7 worried about are guarded at the point of assignment now:
    // forward() and backward() join a finished run before starting a new one,
    // which is what std::terminate() would otherwise be waiting for.
    if (_thread.joinable())
        _thread.join();
    std::vector<Filter *> vec;
    baseFilterBank->getFiltersByType("parallelFilter", vec);
    for (size_t i = 0; i < vec.size(); i++) {
        ((ParallelFilter *)vec[i])->stop();
    }
    return true;
}

void Controller::loopFilters()
{
    toffy::log::debug() << __FUNCTION__;
    if (baseFilterBank->size() == 0) _state = Controller::IDLE;
    // Core owns this name (see guiViewerTypeName in controller.hpp); it used to
    // reach into the viewers module for it. tests/test_layering.cpp pins it.
    int viewers = baseFilterBank->countFiltersByType(guiViewerTypeName);
    // P2-6: was `while (_state > Controller::IDLE)`, which never exits on
    // CERROR (0xff). See Controller::isRunning().
    while (Controller::isRunning(_state)) {
        if (_state == Controller::BACKWARD) f.addData("backward", true);
        baseFilterBank->filter(f, f);
        if (_state == Controller::BACKWARD) f.removeData("backward");
        if (viewers > 0) cv::waitKey(10);
        //cv::waitKey(1);
    }
    cv::destroyAllWindows();
    toffy::log::debug() << "Thread " << __FUNCTION__ << " ends";
}

void Controller::loopFiltersOnce()
{
    toffy::log::debug() << __FUNCTION__;
    if (baseFilterBank->size() == 0) _state = Controller::IDLE;

    // P2-6: same `> IDLE` trap as loopFilters(), and the same fix. This function
    // is currently never called -- its only launcher was the commented-out
    // boost::thread that stepForward()/stepBackward() lost -- so the trap is
    // latent here too, but it is the same enum and the same comparison.
    if (Controller::isRunning(_state)) {
        if (_state == Controller::BACKWARD) f.addData("backward", true);
        baseFilterBank->filter(f, f);
        if (_state == Controller::BACKWARD) f.removeData("backward");
        cv::waitKey(1);
        _state = Controller::IDLE;
    }
    toffy::log::debug() << "Thread " << __FUNCTION__ << " ends";
}

int Controller::loadRuntimeConfig(const std::string &configFile)
{
    toffy::log::debug() << __FUNCTION__;

    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_xml(configFile, pt);

    } catch (boost::property_tree::xml_parser_error &e) {
        toffy::log::error()
            << "FB Could not open config file: " << e.filename() << ". "
            << e.what() << ", in line: " << e.line();
        return -1;
    }

    for (boost::property_tree::ptree::const_iterator it = pt.begin();
         it != pt.end(); ++it) {
        if (FilterFactory::getInstance()->findFilter(it->first))
            FilterFactory::getInstance()->getFilter(it->first)->updateConfig(
                it->second);
        else
            toffy::log::warning()
                << "loadRuntimeConfig: "
                << "Filter " << it->first << " not found.";
    }
    return 1;
}

void Controller::saveRunConfig(std::string fileName)
{
    toffy::log::debug() << __FUNCTION__;
    boost::property_tree::ptree pt;

    pt = baseFilterBank->getConfig();
//std::stringstream ss;
//boost::property_tree::xml_parser::write_xml(ss,pt);
    // The old `#if (BOOST_VERSION > 105500)` guard around this is gone: the
    // `xml_writer_settings<char>` fallback it protected has not been compilable
    // on any supported Boost for years, so the branch was dead code.
    boost::property_tree::xml_parser::xml_writer_settings<std::string> settings(
        '\t', 1);
    boost::property_tree::xml_parser::write_xml(fileName, pt, std::locale(),
                                                settings);
}

int Controller::loadConfigFile(const std::string &configFile)
{
    toffy::log::debug() << __FUNCTION__;

    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_xml(configFile, pt);

    } catch (boost::property_tree::xml_parser_error &e) {
        toffy::log::error()
            << "FB Could not open config file: " << e.filename() << ". "
            << e.what() << ", in line: " << e.line();
        return -1;
    }

    loadPlugins(pt);
    return baseFilterBank->loadConfig(pt, configFile);
}

int Controller::loadConfig(const boost::property_tree::ptree &pt)
{
    toffy::log::debug() << __FUNCTION__;

    loadPlugins(pt);
    return baseFilterBank->loadConfig(pt);
}

void Controller::loadPlugins(const boost::property_tree::ptree &pt)
{
    toffy::log::debug() << __FUNCTION__;
    // P2-10: this used to walk pt.get_child("toffy.plugins") itself, which threw
    // ptree_bad_path on a config with no <plugins> node. One implementation now;
    // see toffy/common/plugins.hpp for what the three versions disagreed about.
    toffy::commons::plugins::loadFromConfig(pt);
}

void Controller::loadPlugin(std::string lib)
{
    // P2-10: the dlopen/dlsym/init sequence now lives in commons::plugins::loadOne,
    // shared with loadPlugins() and with FilterBank. Handles are recorded there,
    // not on Controller::_loads, which is gone -- see getLoadedFilters().
    toffy::commons::plugins::loadOne(lib);
}
