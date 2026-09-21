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
#include <algorithm>
#include <sstream>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <toffy/filter.hpp>
#include <toffy/filter_helpers.hpp>

#include <opencv2/core.hpp>

#include <toffy/filterbank.hpp>

using namespace toffy;
using namespace cv;
namespace logging = boost::log;

std::size_t _filter_counter = 0;

unsigned int Filter::getCounter() const { return _filter_counter; }

Filter::Filter() : _type("Filter.thisShouldNotHappen!") {}

Filter::Filter(std::string type, std::size_t counter /*= -1*/)
    : _type(type),
      _bank(NULL),
      _log_lvl(logging::trivial::info),
      dbg(false),
      update(false)
{
    _filter_counter++;
    if (counter > 0)
        this->_id = _type + "_" + boost::lexical_cast<std::string>(counter);
    else
        this->_id =
            _type + "_" + boost::lexical_cast<std::string>(_filter_counter);
    this->name(this->_id);
#ifdef CM_DEBUG
    _log_lvl = logging::trivial::debug;
#endif
    // setLoggingLvl();
}

void Filter::setLoggingLvl()
{
    // P2-8: this used to be
    //     logging::core::get()->set_filter(logging::trivial::severity >= _log_lvl);
    // i.e. a *per-filter* method reconfigured the severity filter of the whole
    // process. In a pipeline the effective level was therefore whatever the last
    // filter to run happened to want -- order-dependent, and re-evaluated three
    // times per filter per frame by FilterBank::filter().
    //
    // The per-filter level is now plain data: it drives `dbg` here and stays
    // readable via logLvl() / getConfig(). The process-wide severity filter
    // belongs to the application (Player's ctor sets it); use
    // Filter::setGlobalLogLevel() if you need to move it after startup.
    dbg = (_log_lvl <= logging::trivial::debug);
}

void Filter::setGlobalLogLevel(boost::log::trivial::severity_level lvl)
{
    // The single place allowed to touch the shared logging core. Call it once,
    // from the application, not from a filter and never from the frame loop.
    logging::core::get()->set_filter(logging::trivial::severity >= lvl);
}

void Filter::setLogLevel(const std::string& level)
{
    if (level == "debug") {
        _log_lvl = logging::trivial::debug;
    } else if (level == "info") {
        _log_lvl = logging::trivial::info;
    } else if (level == "warn") {
        _log_lvl = logging::trivial::warning;
    } else if (level == "warning") {
        _log_lvl = logging::trivial::warning;
    } else {
        _log_lvl = logging::trivial::info;
    }
    setLoggingLvl();
}

Filter::~Filter() {}

int Filter::loadConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << _type;

    boost::property_tree::ptree::const_assoc_iterator it = pt.find(_type);
    if (it == pt.not_found()) {
        BOOST_LOG_TRIVIAL(error)
            << __FUNCTION__ << " type mismatch instantiating node! "
            << "looked for an XML subtree called " << _type
            << " please check your code, the Filter object seems "
            << "to be have the wrong name!";
        // Stop here. Falling through reached pt.get_child(_type) below, which
        // throws ptree_bad_path: the diagnostic above was always followed by an
        // exception, so a caller relying on the documented return code never
        // saw either the error or a chance to recover.
        return -1;
    }

    const boost::property_tree::ptree& node = pt.get_child(_type);

    _name = node.get("name", _name);
    BOOST_LOG_TRIVIAL(debug) << id() << "::loadConfig NAME SET TO " << _name;

    loadGlobals(node);

    updateConfig(node);

    return 1;
}

int Filter::loadFileConfig(const std::string& configFile)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << _id;
    using boost::property_tree::ptree;
    ptree pt;

    try {
        read_xml(configFile, pt);
    } catch (const boost::property_tree::xml_parser::xml_parser_error& ex) {
        BOOST_LOG_TRIVIAL(error)
            << "error in file " << ex.filename() << " line " << ex.line();
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
    pt.put("options.loglvl", _log_lvl);
    return pt;
}

void Filter::updateConfig(const boost::property_tree::ptree& pt)
{
    // Precedence preserved deliberately: the original applied `loglvl` first and
    // then `options.loglvl` with the previous result as its default, so when both
    // are present `options.loglvl` wins. Collapsing the two into one nested get()
    // inverts that -- keep them sequential.
    const int lvl =
        pt.get<int>("options.loglvl",
                    pt.get<int>("loglvl", static_cast<int>(_log_lvl)));
    // Config values are untrusted: <loglvl>99</loglvl> used to be cast straight
    // into the severity enum, producing a value outside trace..fatal. Clamp it.
    _log_lvl = static_cast<boost::log::trivial::severity_level>(
        std::min(std::max(lvl, static_cast<int>(logging::trivial::trace)),
                 static_cast<int>(logging::trivial::fatal)));
    // Refresh `dbg` from the configured level. Nothing else did: `dbg` was only
    // ever updated when a caller happened to invoke setLoggingLvl(), so a filter
    // configured with <loglvl>debug</loglvl> kept dbg == false until the bank
    // called it on the (now removed) hot path.
    setLoggingLvl();
    pt_optional_get_default(pt, "name", _name, _name);
    BOOST_LOG_TRIVIAL(debug) << id() << "::" << __FUNCTION__ << " name set to "
                            << _name;
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

void Filter::loadGlobals(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    boost::optional<std::string> global =
        pt.get_optional<std::string>("global");
    if (global.is_initialized()) {
        const boost::property_tree::ptree gOptions =
            static_cast<FilterBank*>(_bank)->getBaseFilterbank()->getGlobals(
                *global);

        updateConfig(gOptions);
    } else {
        BOOST_LOG_TRIVIAL(debug) << "No global config.";
    }
}
