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
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <toffy/filter.hpp>
#include <toffy/filter_helpers.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif


#include <toffy/filterbank.hpp>
//#include <toffy/application.hpp>

using namespace toffy;
using namespace cv;
namespace logging = boost::log;

std::size_t _filter_counter = 0;

unsigned int Filter::getCounter() const {return _filter_counter;}

Filter::Filter(): _type("Filter.thisShouldNotHappen!") {
}

Filter::Filter(std::string type, std::size_t counter /*= -1*/):
    _type(type), _bank(NULL), _log_lvl(logging::trivial::info), dbg(false), update(false)
{
    _filter_counter++;
    if (counter > 0)
        this->_id = _type + "_" + boost::lexical_cast<std::string>(counter);
    else
        this->_id = _type + "_" + boost::lexical_cast<std::string>(_filter_counter);
    this->name(this->_id);
#ifdef CM_DEBUG
    _log_lvl = logging::trivial::debug;
#endif
    //setLoggingLvl();
}

void Filter::setLoggingLvl() {
    logging::core::get()->set_filter(logging::trivial::severity >= _log_lvl);
    if (_log_lvl <= 1)
        dbg = true;
    else
        dbg = false;
}

Filter::~Filter() {
}


int Filter::loadConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << _type;

    boost::property_tree::ptree::const_assoc_iterator it = pt.find(_type);
    if ( it == pt.not_found() ) {
        BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << " type mismatch instantiating node! "
                                 << "looked for an XML subtree called " << _type
                                 << " please check your code, the Filter object seems "
                                 << "to be have the wrong name!";
    }

    const boost::property_tree::ptree& node = pt.get_child(_type);

    _name = node.get("name", _name);
    BOOST_LOG_TRIVIAL(debug) << id() << "::loadConfig NAME SET TO " << _name;

    loadGlobals(node);

    updateConfig(node);

    return 1;
}


int Filter::loadFileConfig(const std::string& configFile) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << _id;
    using boost::property_tree::ptree;
    ptree pt;

    try {
        read_xml(configFile, pt);
    } catch (const boost::property_tree::xml_parser::xml_parser_error& ex) {
        BOOST_LOG_TRIVIAL(error) << "error in file "
                                 << ex.filename() << " line " << ex.line();
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
    pt.put("options.loglvl",_log_lvl);
    return pt;
}

void Filter::updateConfig(const boost::property_tree::ptree &pt)
{

    _log_lvl =
            static_cast<boost::log::trivial::severity_level>(
                pt.get<int>("loglvl",_log_lvl) );
    _log_lvl =
            static_cast<boost::log::trivial::severity_level>(
                pt.get<int>("options.loglvl",_log_lvl));
   pt_optional_get_default(pt, "name", _name, _name);
   std::cout << id() << "::updateConfig NAME SET TO " << _name << std::endl; 
}

void Filter::setState(filterState state) {
    this->state = state;
    std::vector<FilterListener*>::iterator it = listeners.begin();
    while (it != listeners.end()) {
        (*it)->stateChanged(*this, state);
        it++;
    }
}

void Filter::removeListener(const FilterListener* l) {
    std::vector<FilterListener*>::iterator it = listeners.begin();
    while (it != listeners.end()) {
        if (*it == l) {
            listeners.erase(it);
            return;
        }
        it++;
    }
}

void Filter::processEvent(Event& /*e*/) {
    BOOST_LOG_TRIVIAL(debug) << id() << " " << __FUNCTION__;
    BOOST_LOG_TRIVIAL(info) << "Filter does not have events declared.";
    return;
}

void Filter::loadGlobals(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    boost::optional< std::string > global =
            pt.get_optional<std::string>("global");
    //std::cout << "global.is_initialized()" << global.is_initialized() << std::endl;
    if (global.is_initialized()) {
        const boost::property_tree::ptree gOptions = static_cast<FilterBank*>(_bank)->getBaseFilterbank()->getGlobals(*global);

        updateConfig(gOptions);
    } else {
        BOOST_LOG_TRIVIAL(debug) << "No global config.";
    }
}
