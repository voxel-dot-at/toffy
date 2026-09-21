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
#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>

#include "toffy/filterbank.hpp"
#include "toffy/filter_helpers.hpp"
#include <toffy/common/plugins.hpp>
#include <iostream>

#ifdef MSVC
#include <windows.h>
#else
#include <dlfcn.h>
#endif

using namespace toffy;
using namespace std;
using namespace cv;

std::size_t FilterBank::_filter_counter = 1;
const std::string FilterBank::id_name = "filterBank";

FilterBank::~FilterBank() { clearBank(); }

bool FilterBank::filter(const Frame& in, Frame& out)
{
    using namespace boost::posix_time;

    // No setLoggingLvl() here or in the loop (P2-8). This used to run the bank's
    // own level, then each child's, then the bank's again after every child --
    // three reconfigurations of the *process-wide* logging core per filter per
    // frame, with the effective level ending up as whichever filter ran last.
    // The logging level is application state now; see Filter::setGlobalLogLevel.
    for (size_t i = 0; i < _pipe.size(); i++) {
        bool success = false;

        ptime start = microsec_clock::local_time();
        try {
            success = _pipe[i]->filter(in, out);

        } catch (std::exception& e) {
            BOOST_LOG_TRIVIAL(error) << name() << "::" << __FUNCTION__
                                     << " Exception in " << _pipe[i]->name();
            BOOST_LOG_TRIVIAL(error)
                << name() << "::" << __FUNCTION__ << " " << e.what();
        }

        boost::posix_time::time_duration diff =
            microsec_clock::local_time() - start;
        if (!success) {
            BOOST_LOG_TRIVIAL(info)
                << id() << "::filter" << i << "\t" << diff.total_milliseconds()
                << "\t" << _pipe[i]->name() << "\t failed!" << endl;
            return false;
        }

        // BOOST_LOG_TRIVIAL(debug)
        //    << id() << "::filter" << i << "\t" << _pipe[i]->name() << "\t done"
        //    << "\t duration: " << diff.total_microseconds() << " us";
    }
    ready.post();
    return true;
}

boost::property_tree::ptree FilterBank::getConfig() const
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << id();
    boost::property_tree::ptree pt;

    for (size_t i = 0; i < _pipe.size(); i++) {
        pt.put_child(_pipe[i]->id(), _pipe[i]->getConfig());
    }
    return pt;
}

int FilterBank::handleConfigItem(
    const std::string& configFile,
    const boost::property_tree::ptree::const_iterator& it)
{
    if (it->first == "filterGroup") {
        // recurse into external config file
        FilterBank* fb = (FilterBank*)ff->createFilter(it->first);
        if (!fb) {
            BOOST_LOG_TRIVIAL(error)
                << "Error creating FilterBank for filterGroup in: "
                << configFile;
            return -1;
        }
        //FilterBank* fb = new FilterBank();
        BOOST_LOG_TRIVIAL(debug) << "FG RECURSE NEW FB #" << it->second.size()
                                 << " " << it->second.data();
        fb->loadFileConfig(it->second.data());
        add(fb);
        // Ignore comments and global options
    } else if (it->first == "<xmlcomment>" || it->first == "globals" ||
               it->first == "plugins") {
        ;
    } else {
        BOOST_LOG_TRIVIAL(debug)
            << "FB LOAD FILTER:: " << it->first << " || " << it->second.size();
        // instantiate new filter instance, configure and add it.
        Filter* f = instantiateFilter(it);
        if (f) {
            add(f);
        } else {
            return 0;
        }
    }
    return 1;
}

Filter* FilterBank::instantiateFilter(
    const boost::property_tree::ptree::const_iterator& it)
{
    BOOST_LOG_TRIVIAL(debug) << "FB INSTANTIATE FILTER:: " << it->first << endl;
    Filter* f = ff->createFilter(it->first);
    if (f) {
        boost::property_tree::ptree pnode;
        pnode.add_child(it->first, it->second);
        string old = f->name();

        BOOST_LOG_TRIVIAL(debug)
            << "FB INSTANTIATE FILTER:: config " << it->first << endl;
        f->bank(this);
        // The return value is deliberately not checked here, and P0-12 does not
        // change that. Making it fatal looks obvious but is not: the codebase
        // has no agreed meaning for a non-positive loadConfig() (finding A23).
        // Cond::loadConfig() returns <= 0 for a merely *empty* cond, which is a
        // valid configuration -- treating it as an error aborts config loading
        // (observed: ctest cond_empty). Propagating failures requires settling
        // A23 first, so it is out of scope for this item.
        f->loadConfig(pnode);
        //ff->renameFilter(f, old, f->name());
        return f;
    } else {
        LOG(warning) " unknown filter " << it->first << " ignored!";
        return 0;
    }
}

int FilterBank::loadConfig(
    const std::string& configFile,
    const boost::property_tree::ptree::const_iterator& begin,
    const boost::property_tree::ptree::const_iterator& end)
{
    BOOST_LOG_TRIVIAL(debug)
        << "FilterBank::" << type() << "::" << __FUNCTION__ << "(begin, end)";
    int errors = 0;
    for (boost::property_tree::ptree::const_iterator it = begin; it != end;
         ++it) {
        BOOST_LOG_TRIVIAL(debug)
            << "FB  iter " << it->first << "\t" << it->second.size();
        int ret = this->handleConfigItem(configFile, it);
        if (!ret) errors++;
    }
    if (errors) {
        LOG(error) << errors
                   << " errors detected! Expect this application to FAIL!";
        // but ok, you're the boss...
        throw std::runtime_error("filterBank::loadConfig() failure");
    }
    return 1;
}

void FilterBank::add(Filter* f)
{
    _pipe.push_back(f);
    //_filters.insert(std::pair<std::string, Filter*>(f->name(),f));
}

int FilterBank::loadConfig(const boost::property_tree::ptree& pt,
                           const std::string& confFile /*= ""*/)
{
    BOOST_LOG_TRIVIAL(debug)
        << __FUNCTION__ << " " << __LINE__ << " " << confFile;
    // const boost::property_tree::ptree& ptRead = pt;
    boost::optional<const boost::property_tree::ptree&> pfilters =
        pt.get_child_optional("toffy");
    //TODO we keep working with configs not enclosed in a root <toffy> for compatibility
    if (!pfilters) {
        BOOST_LOG_TRIVIAL(warning)
            << "Could not open config file: "
            << "Missing root node <toffy>.\n"
            << "Add <toffy> as root node to remove this error.";
        return -1;
    } else {
        const boost::property_tree::ptree& ptRead = *pfilters;
        BOOST_LOG_TRIVIAL(debug)
            << __FUNCTION__ << " first node " << ptRead.begin()->first;
        loadGlobals(ptRead);
        return loadConfig(confFile, ptRead.begin(), ptRead.end());
    }
}

int FilterBank::loadFileConfig(const std::string& confFile)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " FilterBank " << confFile;

    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_xml(confFile, pt);

        BOOST_LOG_TRIVIAL(debug)
            << __FUNCTION__ << " first node " << pt.begin()->first;

    } catch (boost::property_tree::xml_parser_error& e) {
        BOOST_LOG_TRIVIAL(error)
            << "FB Could not open config file: " << e.filename() << ". "
            << e.what() << ", in line: " << e.line();
        return -1;
    }
    // if needed for inherited filterbanks, use FilterBank:: here to avoid confusion with embedded files
    return loadConfig(pt, confFile);
}

const Filter* FilterBank::getFilter(const std::string& name) const
{
    return ff->getFilter(name);
}

const Filter* FilterBank::getFilter(int i) const { return _pipe.at(i); }

Filter* FilterBank::getFilter(const std::string& name)
{
    return ff->getFilter(name);
}

Filter* FilterBank::getFilter(int i) { return _pipe.at(i); }

int FilterBank::getFiltersByType(const std::string& type,
                                 std::vector<Filter*>& vec)
{
    for (std::vector<Filter *>::iterator it=_pipe.begin();
         it < _pipe.end(); it++)
    {
        //BOOST_LOG_TRIVIAL(debug) << type;
        //BOOST_LOG_TRIVIAL(debug) << (*it)->id();
        if ((*it)->type() == "filterBank" ||
            (*it)->type() == "parallelFilter") {
            ((FilterBank*)(*it))->getFiltersByType(type, vec);
        }
        if ((*it)->type() == type) vec.push_back((*it));
    }
    return vec.size();
}

int FilterBank::countFiltersByType(const std::string& type)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    int cnt = 0;
    for (std::vector<Filter*>::iterator it = _pipe.begin(); it < _pipe.end();
         it++) {
        //BOOST_LOG_TRIVIAL(debug) << type;
        //BOOST_LOG_TRIVIAL(debug) << (*it)->id();
        if ((*it)->type() == "filterBank" ||
            (*it)->type() == "parallelFilter") {
            cnt += ((FilterBank*)(*it))->countFiltersByType(type);
        }
        if ((*it)->type() == type) cnt++;
    }
    return cnt;
}

std::optional<int> FilterBank::findPos(const std::string& name) const
{
    for (size_t i = 0; i < _pipe.size(); i++) {
        if (_pipe.at(i)->name() == name) return static_cast<int>(i);
    }
    return std::nullopt;
}

int FilterBank::remove(const std::string& name)
{
    const std::optional<int> pos = findPos(name);
    if (!pos) {
        BOOST_LOG_TRIVIAL(warning)
            << id() << "::remove: no filter named " << name;
        return 0;
    }
    // Erase from the pipeline before the factory deletes the filter, so that
    // _pipe never holds a dangling pointer between the two operations.
    _pipe.erase(_pipe.begin() + *pos);
    ff->deleteFilter(name);
    return 1;
}

int FilterBank::remove(size_t i)
{
    if (i >= _pipe.size()) {
        BOOST_LOG_TRIVIAL(warning) << "Position i: " << i << " out of bounds.";
        return 0;
    }
    const string name = _pipe[i]->name();
    _pipe.erase(_pipe.begin() + i);
    ff->deleteFilter(name);
    return 1;
}

void FilterBank::clearBank()
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    for (size_t i = 0; i < _pipe.size(); i++) {
        string n = _pipe[i]->name();
        try {
            ff->deleteFilter(_pipe[i]->name());
        } catch (std::exception& e) {
            // The shouty `cout << "AU!!!! ..."` that used to follow this carried
            // e.what(), which the log line above did not -- so fold it in here
            // rather than losing the only copy of the diagnostic.
            BOOST_LOG_TRIVIAL(warning)
                << name() << "::clearBank failed at " << i << " " << n
                << " : " << e.what();
        }
    }
    _pipe.clear();
}

void FilterBank::init()
{
    for (size_t i = 0; i < _pipe.size(); i++) {
        _pipe[i]->init();
    }

    Filter::init();
}

void FilterBank::start()
{
    for (size_t i = 0; i < _pipe.size(); i++) {
        _pipe[i]->start();
    }

    Filter::start();
}

void FilterBank::stop()
{
    for (size_t i = 0; i < _pipe.size(); i++) {
        _pipe[i]->stop();
    }

    Filter::stop();
}

FilterBank* FilterBank::getBaseFilterbank()
{
    if (bank() == NULL)
        return this;
    else
        return static_cast<FilterBank*>(bank())->getBaseFilterbank();
}

int FilterBank::loadGlobals(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    boost::optional<const boost::property_tree::ptree&> globals =
        pt.get_child_optional("globals");
    if (!globals) {
        BOOST_LOG_TRIVIAL(info) << "No global configuration included.";
        return 0;
    } else {
        BOOST_LOG_TRIVIAL(info) << "Loading global configuration...";
        _globalConfig = *globals;
        // updateConfig() now refreshes _log_lvl and `dbg` itself, so the separate
        // setLoggingLvl() call that used to follow is gone.
        updateConfig(_globalConfig);
        return 1;
    }
}

const boost::property_tree::ptree& FilterBank::getGlobals(
    std::string node /* = "" */)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    if (node.empty()) {
        return _globalConfig;
    } else {
        boost::optional<boost::property_tree::ptree&> globals =
            _globalConfig.get_child_optional(node.c_str());
        if (!globals) {
            BOOST_LOG_TRIVIAL(warning)
                << "Node " << node << " not present in global configurations.";
            return _globalConfig;
        } else {
            return *globals;
        }
    }
}

int FilterBank::loadPlugins(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BOOST_FOREACH (const boost::property_tree::ptree::value_type& v,
                   pt.get_child("plugins")) {
        BOOST_LOG_TRIVIAL(debug) << "v.first " << v.first;
        BOOST_LOG_TRIVIAL(debug) << "v.second " << v.second.data();

#ifdef MSVC
        HINSTANCE hGetProcIDDLL = LoadLibrary(v.second.data().c_str());

        if (!hGetProcIDDLL) {
            BOOST_LOG_TRIVIAL(warning)
                << "Could not load library: " << v.second.data();
            continue;
        }

        toffy::commons::plugins::init_t init =
            (toffy::commons::plugins::init_t)GetProcAddress(hGetProcIDDLL,
                                                            "init");
        if (!init) {
            BOOST_LOG_TRIVIAL(warning)
                << "Could not load plug-in filters from: " << v.second.data();

#else
        void* libHandle;
        //if(libHandle != NULL) {
        //    dlclose(libHandle);
        //    libHandle = NULL;
        //}

        libHandle = dlopen(v.second.data().c_str(), RTLD_LAZY);
        if (!libHandle) {
            BOOST_LOG_TRIVIAL(warning)
                << "Could not load library: " << v.second.data() << std::endl
                << dlerror();

            // reset errors
            dlerror();
            continue;
        }
        toffy::commons::plugins::init_t init =
            (toffy::commons::plugins::init_t)dlsym(libHandle, "init");
        const char* dlsym_error = dlerror();
        if (dlsym_error) {
            BOOST_LOG_TRIVIAL(warning)
                << "Could not load plug-in filters from: " << v.second.data()
                << std::endl
                << dlerror();
#endif
        } else {
            BOOST_LOG_TRIVIAL(info)
                << "Loaded plug-in filters from: " << v.second.data();
            init(FilterFactory::getInstance());
        }
#ifndef MSVC
        // reset errors
        dlerror();
        //dlclose(libHandle);
#endif
    }
    return 1;
}
