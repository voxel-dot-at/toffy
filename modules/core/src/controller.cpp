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
#include <boost/log/trivial.hpp>

#include <toffy/controller.hpp>
#include <toffy/parallelFilter.hpp>
#include <toffy/common/plugins.hpp>

#include <opencv2/highgui.hpp>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <toffy/viewers/imageview.hpp>

#ifdef MSVC
#include <windows.h>
#else
#include <dlfcn.h>
#endif

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
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    baseFilterBank = static_cast<FilterBank *>(
        toffy::FilterFactory::getInstance()->createFilter("filterBank",
                                                          "baseController"));
    baseFilterBank->bank(NULL);
}

Controller::~Controller()
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    _state = Controller::IDLE;
    toffy::FilterFactory::getInstance()->deleteFilter(baseFilterBank->id());
    baseFilterBank = NULL;
    toffy::FilterFactory::getInstance()->clearCreators();
    /*for (size_t i = 0; i < _loads.size(); i++) {
	cout << "_loads: " << _loads[i] << endl;
	dlclose(_loads[i]);
    }
    _loads.clear();*/
}

bool Controller::forward()
{
    if (_state == Controller::IDLE) {
        std::vector<Filter *> vec;
        baseFilterBank->getFiltersByType("parallelFilter", vec);
        for (size_t i = 0; i < vec.size(); i++) {
            ((ParallelFilter *)vec[i])->init();
            ((ParallelFilter *)vec[i])->start();
        }
        _state = Controller::FORWARD;
        _thread = boost::thread(boost::bind(&Controller::loopFilters, this));
        if (!_thread.joinable()) {
            _state = Controller::IDLE;
            //check thread or the state changed inside
            //SOMETHING WRONG HAPPEND
            //TODO
            return false;
        }
    } else {
        //ALREADY RUNNING
        return false;
    }
    return true;
}

bool Controller::backward()
{
    if (_state == Controller::IDLE) {
        std::vector<Filter *> vec;
        baseFilterBank->getFiltersByType("parallelFilter", vec);
        for (size_t i = 0; i < vec.size(); i++) {
            ((ParallelFilter *)vec[i])->init();
            ((ParallelFilter *)vec[i])->start();
        }
        _state = Controller::BACKWARD;
        _thread = boost::thread(boost::bind(&Controller::loopFilters, this));
        if (!_thread.joinable()) {
            _state = Controller::IDLE;
            //check thread or the state changed inside
            //SOMETHING WRONG HAPPEND
            //TODO
            return false;
        }
    } else {
        //ALREADY RUNNING
        return false;
    }
    return true;
}

bool Controller::stepForward()
{
    if (_state == Controller::IDLE) {
        std::vector<Filter *> vec;
        baseFilterBank->getFiltersByType("parallelFilter", vec);
        for (size_t i = 0; i < vec.size(); i++) {
            ((ParallelFilter *)vec[i])->init();
            ((ParallelFilter *)vec[i])->start();
        }
        /*_state = Controller::FORWARD;
	_thread = boost::thread( boost::bind(&Controller::loopFiltersOnce, this));
	if (!_thread.joinable()) {
	    _state = Controller::IDLE;
	    //check thread or the state changed inside
	    //SOMETHING WRONG HAPPEND
	    //TODO
	    return false;
	}
	_thread.join();*/
        baseFilterBank->filter(f, f);
        for (size_t i = 0; i < vec.size(); i++) {
            ((ParallelFilter *)vec[i])->stop();
        }
    } else {
        //already RUNNING
        return false;
    }
    return true;
}

bool Controller::stedBackward()
{
    if (_state == Controller::IDLE) {
        std::vector<Filter *> vec;
        baseFilterBank->getFiltersByType("parallelFilter", vec);
        for (size_t i = 0; i < vec.size(); i++) {
            ((ParallelFilter *)vec[i])->init();
            ((ParallelFilter *)vec[i])->start();
        }
        /*_state = Controller::BACKWARD;
	_thread = boost::thread( boost::bind(&Controller::loopFiltersOnce, this));
	if (!_thread.joinable()) {
	    _state = Controller::IDLE;
	    //check thread or the state changed inside
	    //SOMETHING WRONG HAPPEND
	    //TODO
	    return false;
	}*/
        f.addData("backward", true);
        baseFilterBank->filter(f, f);
        f.removeData("backward");
        for (size_t i = 0; i < vec.size(); i++) {
            ((ParallelFilter *)vec[i])->stop();
        }
    } else {
        //already RUNNING
        return false;
    }
    //f.removeData("backward");
    return true;
}

bool Controller::stop()
{
    _state = Controller::IDLE;
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
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    if (baseFilterBank->size() == 0) _state = Controller::IDLE;
    int viewers = baseFilterBank->countFiltersByType(ImageView::id_name);
    while (_state > Controller::IDLE) {
        if (_state == Controller::BACKWARD) f.addData("backward", true);
        baseFilterBank->filter(f, f);
        if (_state == Controller::BACKWARD) f.removeData("backward");
        if (viewers > 0) cv::waitKey(10);
        //cv::waitKey(1);
    }
    cv::destroyAllWindows();
    BOOST_LOG_TRIVIAL(debug) << "Thread " << __FUNCTION__ << " ends";
}

void Controller::loopFiltersOnce()
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    if (baseFilterBank->size() == 0) _state = Controller::IDLE;

    if (_state > Controller::IDLE) {
        if (_state == Controller::BACKWARD) f.addData("backward", true);
        std::cout << "in thread." << std::endl;
        baseFilterBank->filter(f, f);
        if (_state == Controller::BACKWARD) f.removeData("backward");
        cv::waitKey(1);
        _state = Controller::IDLE;
    }
    BOOST_LOG_TRIVIAL(debug) << "Thread " << __FUNCTION__ << " ends";
    std::cout << "Thread ends." << std::endl;
}

int Controller::loadRuntimeConfig(const std::string &configFile)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_xml(configFile, pt);

    } catch (boost::property_tree::xml_parser_error &e) {
        BOOST_LOG_TRIVIAL(error)
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
            BOOST_LOG_TRIVIAL(warning)
                << "loadRuntimeConfig: "
                << "Filter " << it->first << " not found.";
    }
    return 1;
}

void Controller::saveRunConfig(std::string fileName)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    boost::property_tree::ptree pt;

    pt = baseFilterBank->getConfig();
//std::stringstream ss;
//boost::property_tree::xml_parser::write_xml(ss,pt);
#if (BOOST_VERSION > 105500)
    boost::property_tree::xml_parser::xml_writer_settings<std::string> settings(
        '\t', 1);
#else
    boost::property_tree::xml_parser::xml_writer_settings<char> settings('\t',
                                                                         1);
#endif
    boost::property_tree::xml_parser::write_xml(fileName, pt, std::locale(),
                                                settings);
    //std::cout << "OUTPUT: " << ss.str() << std::endl;
}

int Controller::loadConfigFile(const std::string &configFile)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_xml(configFile, pt);

    } catch (boost::property_tree::xml_parser_error &e) {
        BOOST_LOG_TRIVIAL(error)
            << "FB Could not open config file: " << e.filename() << ". "
            << e.what() << ", in line: " << e.line();
        return -1;
    }

    loadPlugins(pt);
    return baseFilterBank->loadConfig(pt, configFile);
}

int Controller::loadConfig(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    loadPlugins(pt);
    return baseFilterBank->loadConfig(pt);
}

void Controller::loadPlugins(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BOOST_FOREACH (const boost::property_tree::ptree::value_type &v,
                   pt.get_child("toffy.plugins")) {
        BOOST_LOG_TRIVIAL(debug) << "v.first " << v.first;
        BOOST_LOG_TRIVIAL(debug) << "v.second " << v.second.data();

        //cout << "v.first " << v.first << endl;
        //cout << "v.second " << v.second.data() << endl;
        loadPlugin(v.second.data());
    }

    return;
}

void Controller::loadPlugin(std::string lib)
{
    /*if(libHandle != NULL) {
	dlclose(libHandle);
	libHandle = NULL;
    }*/
#ifdef MSVC
    HINSTANCE hGetProcIDDLL = LoadLibrary(lib.c_str());

    if (hGetProcIDDLL == NULL) {
        BOOST_LOG_TRIVIAL(warning) << "Could not load library: " << lib.c_str();
        return;
    }

    toffy::commons::plugins::init_t init =
        (toffy::commons::plugins::init_t)GetProcAddress(hGetProcIDDLL, "init");
    if (init == NULL) {
        BOOST_LOG_TRIVIAL(warning)
            << "Could not load plug-in filters from: " << lib.c_str();

#else
    void *libHandle;
    std::cout << "lib: " << lib << std::endl;
    libHandle = dlopen(lib.c_str(), RTLD_LAZY);
    if (!libHandle) {
        BOOST_LOG_TRIVIAL(warning)
            << "Could not load library: " << lib << std::endl
            << dlerror();

        // reset errors
        dlerror();
        return;
    }
    toffy::commons::plugins::init_t init =
        (toffy::commons::plugins::init_t)dlsym(libHandle, "init");
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
        BOOST_LOG_TRIVIAL(warning)
            << "Could not load plug-in filters from: " << lib << std::endl
            << dlsym_error;
#endif
    } else {
        // use it to do the calculation
        BOOST_LOG_TRIVIAL(info) << "Loaded plug-in filters from: " << lib;
        // use it to do the calculation
        std::cout << "Calling hello...\n";
        init(FilterFactory::getInstance());

#ifdef MSVC
        _loads.push_back(hGetProcIDDLL);
#else
        _loads.push_back(libHandle);
        libHandle = NULL;
#endif
    }
#ifndef MSVC
    // reset errors
    dlerror();
    //dlclose(libHandle);
#endif
}
