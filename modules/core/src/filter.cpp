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
    : _type(type),
      _bank(NULL),
      _logLevel(toffy::log::level::info),
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
    _logLevel = toffy::log::level::debug;
#endif
}

void Filter::setGlobalLogLevel(toffy::log::level lvl)
{
    // The single place allowed to move the process-wide level. Call it once,
    // from the application, not from a filter and never from the frame loop.
    //
    // P2-8 removed the per-filter version of this, which called
    // logging::core::get()->set_filter() from a method FilterBank::filter()
    // ran three times per child per frame: the effective level was whatever the
    // last filter to run wanted, and it was re-decided every frame.
    toffy::log::root().setLevel(lvl);
}

void Filter::setLogLevel(const std::string& level)
{
    // Unparseable falls back to info, which is what the if-chain this replaced
    // did. "warn" is accepted because pipeline files written over the last
    // decade use it.
    _logLevel = toffy::log::parseLevel(level, toffy::log::level::info);
}

toffy::log::record Filter::filterRecord(toffy::log::level lvl,
                                       const char* func) const
{
    // One channel per filter *type*, not per instance: a pipeline can hold
    // hundreds of instances of one class and they all want the same knob, while
    // the instance is already identified by the id() in the prefix.
    if (_channel == nullptr) {
        _channel = &toffy::log::getLogger("toffy.filters." + type());
    }
    toffy::log::record rec = _channel->message(lvl);
    rec << id() << "::" << func << "() : ";
    return rec;
}

Filter::~Filter() {}

int Filter::loadConfig(const boost::property_tree::ptree& pt)
{
    toffy::log::debug() << __FUNCTION__ << " " << _type;

    boost::property_tree::ptree::const_assoc_iterator it = pt.find(_type);
    if (it == pt.not_found()) {
        toffy::log::error()
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
    toffy::log::debug() << id() << "::loadConfig NAME SET TO " << _name;

    loadGlobals(node);

    updateConfig(node);

    return 1;
}

int Filter::loadFileConfig(const std::string& configFile)
{
    toffy::log::debug() << __FUNCTION__ << _id;
    using boost::property_tree::ptree;
    ptree pt;

    try {
        read_xml(configFile, pt);
    } catch (const boost::property_tree::xml_parser::xml_parser_error& ex) {
        toffy::log::error()
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
    // Explicit int: `level` is a scoped enum, so it no longer streams itself
    // the way the unscoped Boost severity did. The number in the file is the
    // same 0..5 it always was.
    pt.put("options.loglvl", static_cast<int>(_logLevel));
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
                    pt.get<int>("loglvl", static_cast<int>(_logLevel)));
    // Config values are untrusted: <loglvl>99</loglvl> used to be cast straight
    // into the severity enum, producing a value outside trace..fatal. Clamp it.
    // `off` is deliberately not reachable from config -- it is an application
    // decision, not a pipeline one.
    _logLevel = static_cast<toffy::log::level>(
        std::min(std::max(lvl, static_cast<int>(toffy::log::level::trace)),
                 static_cast<int>(toffy::log::level::fatal)));
    pt_optional_get_default(pt, "name", _name, _name);
    toffy::log::debug() << id() << "::" << __FUNCTION__ << " name set to "
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
    toffy::log::debug() << __FUNCTION__;
    boost::optional<std::string> global =
        pt.get_optional<std::string>("global");
    if (global.is_initialized()) {
        const boost::property_tree::ptree gOptions =
            static_cast<FilterBank*>(_bank)->getBaseFilterbank()->getGlobals(
                *global);

        updateConfig(gOptions);
    } else {
        toffy::log::debug() << "No global config.";
    }
}
