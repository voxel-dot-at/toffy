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


#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>

#include "toffy/filterbank.hpp"
#include "toffy/filter_helpers.hpp"
#include "toffy/event.hpp"
#include <toffy/common/plugins.hpp>
#include <toffy/logging.hpp>

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

    setLoggingLvl();  // set our own log level..
    for (size_t i = 0; i < _pipe.size(); i++) {
        bool success = false;

        _pipe[i]->setLoggingLvl();
        ptime start = microsec_clock::local_time();
        try {
            success = _pipe[i]->filter(in, out);

        } catch (std::exception& e) {
            LOGE << name() << "::" << __FUNCTION__ << " Exception in "
                 << _pipe[i]->name();
            LOGE << name() << "::" << __FUNCTION__ << " " << e.what();
        }

        boost::posix_time::time_duration diff =
            microsec_clock::local_time() - start;
        setLoggingLvl();
        if (!success) {
            LOGI << id() << "::filter" << i << "\t" << diff.total_milliseconds()
                 << "\t" << _pipe[i]->name() << "\t failed!";
            return false;
        }
    }
    ready.post();
    return true;
}

boost::property_tree::ptree FilterBank::getConfig() const
{
    LOGD << __FUNCTION__ << id();
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
            LOGE << "Error creating FilterBank for filterGroup in: "
                 << configFile;
            return -1;
        }
        //FilterBank* fb = new FilterBank();
        LOGD << "FG RECURSE NEW FB #" << it->second.size() << " "
             << it->second.data();
        fb->loadFileConfig(it->second.data());
        add(fb);
        // Ignore comments and global options
    } else if (it->first == "<xmlcomment>" || it->first == "globals" ||
               it->first == "plugins") {
        ;
    } else {
        LOGD << "FB LOAD FILTER:: " << it->first << " || " << it->second.size();
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
    LOGD << "FB INSTANTIATE FILTER:: " << it->first;
    Filter* f = ff->createFilter(it->first);
    if (f) {
        boost::property_tree::ptree pnode;
        pnode.add_child(it->first, it->second);
        string old = f->name();

        LOGD << "FB INSTANTIATE FILTER:: config " << it->first;
        f->bank(this);
        f->loadConfig(pnode);
        //ff->renameFilter(f, old, f->name());
        return f;
    } else {
        LOG(warning) << " unknown filter " << it->first << " ignored!";
        return 0;
    }
}

int FilterBank::loadConfig(
    const std::string& configFile,
    const boost::property_tree::ptree::const_iterator& begin,
    const boost::property_tree::ptree::const_iterator& end)
{
    LOGD << "FilterBank::" << type() << "::" << __FUNCTION__ << "(begin, end)";
    int errors = 0;
    for (boost::property_tree::ptree::const_iterator it = begin; it != end;
         ++it) {
        LOGD << "FB  iter " << it->first << "\t" << it->second.size();
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
    LOGD << __FUNCTION__ << " " << __LINE__ << " " << confFile;
    // const boost::property_tree::ptree& ptRead = pt;
    boost::optional<const boost::property_tree::ptree&> pfilters =
        pt.get_child_optional("toffy");
    //TODO we keep working with configs not enclosed in a root <toffy> for compatibility
    if (!pfilters) {
        LOGW << "Could not open config file: "
             << "Missing root node <toffy>.\n"
             << "Add <toffy> as root node to remove this error.";
        return -1;
    } else {
        const boost::property_tree::ptree& ptRead = *pfilters;
        LOGD << __FUNCTION__ << " first node " << ptRead.begin()->first;
        loadGlobals(ptRead);
        return loadConfig(confFile, ptRead.begin(), ptRead.end());
    }
}

int FilterBank::loadFileConfig(const std::string& confFile)
{
    LOGD << __FUNCTION__ << " FilterBank " << confFile;

    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_xml(confFile, pt);

        LOGD << __FUNCTION__ << " first node " << pt.begin()->first;

    } catch (boost::property_tree::xml_parser_error& e) {
        LOGE << "FB Could not open config file: " << e.filename() << ". "
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
    for (std::vector<Filter*>::iterator it = _pipe.begin(); it < _pipe.end();
         it++) {
        //LOGD << type;
        //LOGD << (*it)->id();
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
    LOGD << __FUNCTION__;
    int cnt = 0;
    for (std::vector<Filter*>::iterator it = _pipe.begin(); it < _pipe.end();
         it++) {
        //LOGD << type;
        //LOGD << (*it)->id();
        if ((*it)->type() == "filterBank" ||
            (*it)->type() == "parallelFilter") {
            cnt += ((FilterBank*)(*it))->countFiltersByType(type);
        }
        if ((*it)->type() == type) cnt++;
    }
    return cnt;
}

size_t FilterBank::findPos(std::string name)
{
    for (size_t i = 0; i < _pipe.size(); i++) {
        if (_pipe.at(i)->name() == name) return i;
    }
    return -1;
}

int FilterBank::remove(std::string name)
{
    int pos = findPos(name);
    _pipe.erase(_pipe.begin() + pos);
    ff->deleteFilter(name);
    return 1;
}

int FilterBank::remove(size_t i)
{
    if (i >= _pipe.size()) {
        LOGW << "Position i: " << i << "out of bounds.";
        return -1;
    }
    const string name = _pipe[i]->name();
    ff->deleteFilter(name);
    _pipe.erase(_pipe.end() + i);
    return 1;
}

void FilterBank::clearBank()
{
    LOGD << __FUNCTION__;
    for (size_t i = 0; i < _pipe.size(); i++) {
        string n = _pipe[i]->name();
        try {
            ff->deleteFilter(_pipe[i]->name());
        } catch (std::exception& e) {
            LOGW << name() << "::clearBank failed at " << i << " " << n;

            cout << "AU!!!! clearBank failed with: "
                 << " ****** " << e.what() << endl;
        }
    }
    _pipe.clear();
}

void FilterBank::processEvent(Event& e)
{
    if (e.receiverType() == Event::FILTER) {
        Filter* f = ff->getFilter(e.receiver());
        f->processEvent(e);
    } else {  //Event::ReceiverType::FILTER_TYPE
        std::vector<Filter*> fg;
        ff->getFiltersByType(e.receiver(), fg);
        for (size_t i = 0; i < fg.size(); i++) fg[i]->processEvent(e);
    }
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
        _pipe[i]->start();
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
    LOGD << __FUNCTION__;
    boost::optional<const boost::property_tree::ptree&> globals =
        pt.get_child_optional("globals");
    if (!globals) {
        LOGI << "No global configuration included.";
        return 0;
    } else {
        LOGI << "Loading global configuration...";
        _globalConfig = *globals;
        updateConfig(_globalConfig);
        setLoggingLvl();
        return 1;
    }
}

const boost::property_tree::ptree& FilterBank::getGlobals(
    std::string node /* = "" */)
{
    LOGD << __FUNCTION__;
    if (node.empty()) {
        return _globalConfig;
    } else {
        boost::optional<boost::property_tree::ptree&> globals =
            _globalConfig.get_child_optional(node.c_str());
        if (!globals) {
            LOGW << "Node " << node << " not present in global configurations.";
            return _globalConfig;
        } else {
            return *globals;
        }
    }
}

int FilterBank::loadPlugins(const boost::property_tree::ptree& pt)
{
    LOGD << __FUNCTION__;
    BOOST_FOREACH (const boost::property_tree::ptree::value_type& v,
                   pt.get_child("plugins")) {
        LOGD << "v.first " << v.first;
        LOGD << "v.second " << v.second.data();

        cout << "v.first " << v.first << endl;
        cout << "v.second " << v.second.data() << endl;

#ifdef MSVC
        HINSTANCE hGetProcIDDLL = LoadLibrary(v.second.data().c_str());

        if (!hGetProcIDDLL) {
            LOGW << "Could not load library: " << v.second.data();
            continue;
        }

        toffy::commons::plugins::init_t init =
            (toffy::commons::plugins::init_t)GetProcAddress(hGetProcIDDLL,
                                                            "init");
        if (!init) {
            LOGW << "Could not load plug-in filters from: " << v.second.data();

#else
        void* libHandle;
        //if(libHandle != NULL) {
        //    dlclose(libHandle);
        //    libHandle = NULL;
        //}

        libHandle = dlopen(v.second.data().c_str(), RTLD_LAZY);
        if (!libHandle) {
            LOGW << "Could not load library: " << v.second.data() << " "
                 << dlerror();

            // reset errors
            dlerror();
            continue;
        }
        toffy::commons::plugins::init_t init =
            (toffy::commons::plugins::init_t)dlsym(libHandle, "init");
        const char* dlsym_error = dlerror();
        if (dlsym_error) {
            LOGW << "Could not load plug-in filters from: " << v.second.data()
                 << " " << dlerror();
#endif
        } else {
            // use it to do the calculation
            LOGI << "Loaded plug-in filters from: " << v.second.data();
            // use it to do the calculation
            std::cout << "Calling init...\n";
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
