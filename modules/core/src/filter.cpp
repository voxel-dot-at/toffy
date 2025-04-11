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
#include <sstream>

#include <boost/log/core.hpp>

#include <boost/log/expressions.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <toffy/filter.hpp>
#include <toffy/filter_helpers.hpp>

#include <opencv2/core.hpp>

#include <toffy/filterbank.hpp>

using namespace toffy;
using namespace cv;

std::size_t _filter_counter = 0;

unsigned int Filter::getCounter() const { return _filter_counter; }

Filter::Filter() : _type("Filter.thisShouldNotHappen!") {}

Filter::Filter(std::string type, std::size_t counter /*= -1*/)
    : _type(type), _bank(NULL), dbg(false), update(false)
{
    _filter_counter++;
    logger.setLevel(toffy::log::info);
    if (counter > 0)
        this->_id = _type + "_" + std::to_string(counter);
    else
        this->_id =
            _type + "_" + std::to_string(_filter_counter);
    this->name(this->_id);
#ifdef CM_DEBUG
    _log_lvl = logging::trivial::debug;
#endif
    // setLoggingLvl();
}

void Filter::setLoggingLvl()
{
    if (logger.getLevel() <= toffy::log::debug)
        dbg = true;
    else
        dbg = false;
}

void Filter::setLogLevel(const std::string& level)
{
    toffy::log::logLevel _log_lvl;
    if (level == "debug") {
        _log_lvl = toffy::log::debug;
    } else if (level == "info") {
        _log_lvl = toffy::log::info;
    } else if (level == "warn") {
        _log_lvl = toffy::log::warning;
    } else if (level == "warning") {
        _log_lvl = toffy::log::warning;
    } else {
        _log_lvl = toffy::log::info;
    }
    this->logger.setLevel(_log_lvl);
    // setLoggingLvl();
}

Filter::~Filter() {}

int Filter::loadConfig(const boost::property_tree::ptree& pt)
{
    LOGD << __FUNCTION__ << " " << _type;

    boost::property_tree::ptree::const_assoc_iterator it = pt.find(_type);
    if (it == pt.not_found()) {
        LOGE << __FUNCTION__ << " type mismatch instantiating node! "
             << "looked for an XML subtree called " << _type
             << " please check your code, the Filter object seems "
             << "to be have the wrong name!";
    }

    const boost::property_tree::ptree& node = pt.get_child(_type);

    _name = node.get("name", _name);
    LOGD << id() << "::loadConfig NAME SET TO " << _name;

    loadGlobals(node);

    updateConfig(node);

    return 1;
}

int Filter::loadFileConfig(const std::string& configFile)
{
    LOGD << __FUNCTION__ << _id;
    using boost::property_tree::ptree;
    ptree pt;

    try {
        read_xml(configFile, pt);
    } catch (const boost::property_tree::xml_parser::xml_parser_error& ex) {
        LOGE << "error in file " << ex.filename() << " line " << ex.line();
        return -1;
    }
    return loadConfig(pt);
}

boost::property_tree::ptree Filter::getConfig() const
{
    boost::property_tree::ptree pt;
    pt.put("name", name());
    pt.put("type", type());
    pt.put("id", id());
    pt.put("options.loglevel", logger.getLevel());
    return pt;
}

void Filter::updateConfig(const boost::property_tree::ptree& pt)
{
    using namespace std;

    toffy::log::logLevel _log_lvl = static_cast<toffy::log::logLevel>(
        pt.get<int>("loglvl", logger.getLevel()));  // @deprecated!
    if (pt.find("loglvl") != pt.not_found()) {
        cout << "DEPRECATED! FIX loglvl to options.logLevel for " << name()
             << endl;
    }
    _log_lvl = static_cast<toffy::log::logLevel>(
        pt.get<int>("options.loglvl", _log_lvl));
    if (pt.find("options.loglvl") != pt.not_found()) {
        cout << "DEPRECATED! FIX options.loglvl to options.logLevel for "
             << name() << endl;
    }
    _log_lvl = static_cast<toffy::log::logLevel>(
        pt.get<int>("options.logLevel", _log_lvl));

    pt_optional_get_default(pt, "name", _name, _name);
    std::cout << id() << "::updateConfig NAME SET TO " << _name << std::endl;
}

void Filter::setState(filterState state)
{
    this->state = state;
    std::vector<FilterListener*>::iterator it = listeners.begin();
    while (it != listeners.end()) {
        (*it)->stateChanged(*this, state);
        it++;
    }
}

void Filter::removeListener(const FilterListener* l)
{
    std::vector<FilterListener*>::iterator it = listeners.begin();
    while (it != listeners.end()) {
        if (*it == l) {
            listeners.erase(it);
            return;
        }
        it++;
    }
}

void Filter::processEvent(Event& /*e*/)
{
    LOGD << id() << " " << __FUNCTION__;
    LOGI << "Filter does not have events declared.";
    return;
}

void Filter::loadGlobals(const boost::property_tree::ptree& pt)
{
    LOGD << __FUNCTION__;
    boost::optional<std::string> global =
        pt.get_optional<std::string>("global");
    // std::cout << "global.is_initialized()" << global.is_initialized() <<
    // std::endl;
    if (global.is_initialized()) {
        const boost::property_tree::ptree gOptions =
            static_cast<FilterBank*>(_bank)->getBaseFilterbank()->getGlobals(
                *global);

        updateConfig(gOptions);
    } else {
        LOGD << "No global config.";
    }
}
