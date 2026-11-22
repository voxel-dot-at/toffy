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

using namespace toffy;

Player::Player(): Player(toffy::log::level::debug, false) {
}

Player::Player(toffy::log::level severity, bool file) {
    // initOnce, not init: an application that configured logging in main()
    // before constructing a Player must not have its configuration replaced by
    // this constructor's defaults. P2-17.
    toffy::log::config cfg;
    cfg.minLevel = severity;
    cfg.fileSink = file;
    toffy::log::initOnce(cfg);
}

Player::~Player() {
    toffy::log::debug() << __FUNCTION__;
    // Used to call logging::core::get()->remove_all_sinks() here, which silenced
    // logging for the *whole process* the moment one Player went out of scope --
    // any filter or any other Player still running lost its output. Sinks are
    // application-owned; the application tears them down, not a library object.
}

void Player::loadFilter(std::string name, CreateFilterFn fn) {
    FilterFactory::getInstance()->registerCreator(name,fn);
}

int Player::loadConfig(const std::string &configFile) {
    toffy::log::debug() << __FUNCTION__;

    boost::property_tree::ptree pt;
    try {
        boost::property_tree::read_xml(configFile, pt);

    } catch (boost::property_tree::xml_parser_error &e) {
        toffy::log::error() << "FB Could not open config file: "
                                 << e.filename() << ". " << e.what()
                                 << ", in line: " << e.line();
        return -1;
    }

    loadPlugins(pt);
    return _controller.baseBank()->loadConfig(pt,configFile);
}

bool Player::hasKey(const std::string& key) const
{
    return _controller.getFrame().hasKey(key);
}

boost::any Player::getData(const std::string& key) {
    return _controller.getFrame().getData(key);
}

void Player::loadData(std::string key, boost::any data) {
    _controller.getFrame().addData(key,data);
}

void Player::removeData(const std::string& key) {
    _controller.getFrame().removeData(key);
}

void Player::runOnce() {
    _controller.stepForward();
}

void Player::run() {
    toffy::log::debug() << __FUNCTION__;
    _controller.forward();

}

// Declared since the first release but never defined, so every caller got an
// undefined-reference link error (finding A16). Delegates to Controller::stop();
// the public signature stays void, so Controller's bool is intentionally not
// surfaced -- changing it would be an API change, which is out of scope here.
void Player::stop() {
    toffy::log::debug() << __FUNCTION__;
    _controller.stop();
}

void Player::loadPlugin(std::string lib) {

    return _controller.loadPlugin(lib);
}

void Player::loadPlugins(const boost::property_tree::ptree& pt) {
    toffy::log::debug() << __FILE__ << " " << __FUNCTION__ ;

    try {
        BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, pt.get_child("toffy.plugins")) {

            toffy::log::debug() << "v.first " << v.first;
            toffy::log::debug() << "v.second " << v.second.data();

            _controller.loadPlugin(v.second.data());
        }
    } catch (std::exception& e) {
        toffy::log::warning() << "Exception while loading plugins! " << e.what();
    }

    return;
}
