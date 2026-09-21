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
#include <toffy/player.hpp>
#include <toffy/filterfactory.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup.hpp>

using namespace toffy;

namespace logging = boost::log;
//namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

Player::Player(): Player(logging::trivial::severity_level::debug, false) {
}

Player::Player(logging::trivial::severity_level severity, bool file) {
    if (file) {
        // Only a file sink is added here, so constructing Player with file=true
        // sends nothing to the console. That is existing behaviour, noted because
        // the commented-out add_console_log() that used to sit here made it look
        // like console output was meant to be added alongside.
        //
        // Installed at most once per process (P2-8). Two things force this:
        //   * ~Player no longer calls remove_all_sinks() (that silenced logging
        //     for the whole process), so without a guard sequential Players
        //     would accumulate a file sink each and duplicate every record;
        //   * add_file_log() builds a *new* sink on each call, so Boost's own
        //     "already registered, call ignored" rule does not help.
        // Boost.Log as built here exposes no way to enumerate the core's sinks
        // (only add_sink/remove_sink/remove_all_sinks), so the flag has to live
        // on this side rather than being queried from the logging core.
        static bool file_sink_installed = false;
        if (!file_sink_installed) {
            file_sink_installed = true;
            logging::add_file_log
                (
                    keywords::file_name = "toffy_%N.log", /*< file name pattern >*/
                    keywords::rotation_size = 10 * 1024 * 1024,/*< rotate files every 10 MiB... >*/
                    keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0), /*< ...or at midnight >*/
                    keywords::format = "[%TimeStamp%]: %Message%", /*< log record format >*/
                    keywords::auto_flush = true,
                    keywords::open_mode = (std::ios::out | std::ios::app),
                    keywords::severity = logging::trivial::debug
                );
        }
    }
    logging::core::get()->set_filter(logging::trivial::severity >= severity);
    logging::add_common_attributes();
}

Player::~Player() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    // Used to call logging::core::get()->remove_all_sinks() here, which silenced
    // logging for the *whole process* the moment one Player went out of scope --
    // any filter or any other Player still running lost its output. Sinks are
    // application-owned; the application tears them down, not a library object.
}

void Player::loadFilter(std::string name, CreateFilterFn fn) {
    FilterFactory::getInstance()->registerCreator(name,fn);
}

int Player::loadConfig(const std::string &configFile) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_xml(configFile, pt);

    } catch (boost::property_tree::xml_parser_error &e) {
        BOOST_LOG_TRIVIAL(error) << "FB Could not open config file: "
                                 << e.filename() << ". " << e.what()
                                 << ", in line: " << e.line();
        return -1;
    }

    loadPlugins(pt);
    return _controller.baseFilterBank->loadConfig(pt,configFile);
}

bool Player::hasKey(const std::string& key) const
{
    return _controller.f.hasKey(key);
}

boost::any Player::getData(const std::string& key) {
    return _controller.f.getData(key);
}

void Player::loadData(std::string key, boost::any data) {
    _controller.f.addData(key,data);
}

void Player::removeData(const std::string& key) {
    _controller.f.removeData(key);
}

void Player::runOnce() {
    _controller.stepForward();
}

void Player::run() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    _controller.forward();

}

// Declared since the first release but never defined, so every caller got an
// undefined-reference link error (finding A16). Delegates to Controller::stop();
// the public signature stays void, so Controller's bool is intentionally not
// surfaced -- changing it would be an API change, which is out of scope here.
void Player::stop() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    _controller.stop();
}

void Player::loadPlugin(std::string lib) {

    return _controller.loadPlugin(lib);
}

void Player::loadPlugins(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FILE__ << " " << __FUNCTION__ ;

    try {
        BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, pt.get_child("toffy.plugins")) {

            BOOST_LOG_TRIVIAL(debug) << "v.first " << v.first;
            BOOST_LOG_TRIVIAL(debug) << "v.second " << v.second.data();

            _controller.loadPlugin(v.second.data());
        }
    } catch (std::exception& e) {
        BOOST_LOG_TRIVIAL(warning) << "Exception while loading plugins! " << e.what();
    }

    return;
}
