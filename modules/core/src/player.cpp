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

Player::Player(logging::trivial::severity_level severity, bool file) {
    if (file) {
        /*logging::add_console_log(
            std::cout,
            keywords::format = "[%TimeStamp%]: %Message%",
            keywords::auto_flush = true,
            keywords::severity = logging::trivial::info
        );*/
        // Output message to file
        logging::add_file_log
                (
                    keywords::file_name = "Toffy_%N.log", /*< file name pattern >*/
                    keywords::rotation_size = 5 * 1024 * 1024,/*< rotate files every 10 MiB... >*/
                    keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0), /*< ...or at midnight >*/
                    keywords::format = "[%TimeStamp%]: %Message%", /*< log record format >*/
                    keywords::auto_flush = true,
                    keywords::open_mode = (std::ios::out | std::ios::app),
                    keywords::severity = logging::trivial::debug
                );
    }
    logging::core::get()->set_filter(logging::trivial::severity >= severity);
    logging::add_common_attributes();
}

Player::~Player() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    logging::core::get()->remove_all_sinks();
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
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    _controller.stepForward();

}

void Player::run() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    _controller.forward();

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
