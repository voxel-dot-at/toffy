#pragma once

#include <toffy/toffy_export.h>
#include <toffy/toffy_config.h>

#include <toffy/controller.hpp>

#include <toffy/filterfactory.hpp>

#include <boost/thread.hpp>
#include <boost/log/trivial.hpp>

namespace toffy {
/**
 * @brief Player is the start point utility for toffy. It keeps the application level objects and allows the creation of you filter through a config file.
 * @ingroup Core
 *
 * Adds the application logic to use toffy
 *
 * @todo Player is still not independent of the web control module. For run the
 * filters it uses the controller, sending http request objects with the actions
 * coded in JSON in the content. This should be change, maybe creating a c++ Api
 * that may be also use by the controller.
 */
class TOFFY_EXPORT Player {
public:
    /**
     * @brief Constructor that sets the debug filter
     * @param severity Boost severity_level
     * @param file Flag for logging to a file
     *
     * A creation time we can select the debug level and if we want to save the
     * log to a file. By default the info level is set.
     *
     */
    Player(boost::log::trivial::severity_level severity =
	    boost::log::trivial::info,bool file=false);

    virtual ~Player();

    /**
     * @brief Read the toffy config file and instanciate all filters
     * @param configFile XML file with toffy filters
     * @return Positive on success, negative or 0 in failed
     */
    int loadConfig(const std::string &configFile);

    /**
     * @brief An accessor to the Frame to load data from different sources
     * @param key Data identifier
     * @param data
     */
    void loadData(std::string key, boost::any data);

    /**
     * @brief Query the frame if data is available under "key" 
     * @param key
     * @return true if frame contains this key
     */
    bool hasKey(const std::string& key) const;

    /**
     * @brief Retrieves data from frame
     * @param key
     * @return Data on success, empty if failed
     */
    boost::any getData(const std::string& key);

    /**
     * @brief Removes data from the Frame
     * @param key Data identifier
     */
    void removeData(const std::string& key);

    /**
     * @brief Do an iteration through all filters
     */
    void runOnce();

    /**
     * @brief Loop over all filters in a separate thread
     */
    void run();

    /**
     * @brief Stops in exist the execution filter thread
     */
    void stop();

    /**
     * @brief Adds external toffy filter at runtime
     * @param name Filter identifier
     * @param fn Pointer to the Filter creation function
     *
     */
    void loadFilter(std::string name, CreateFilterFn fn);

    void loadPlugin(std::string lib);

    /**
     * @brief Getter for the base FilterBank
     * @return Pointer to base Filterbank on success, NULL in failed
     */
    FilterBank* filterBank() {return _controller.baseFilterBank;}

    Controller * getController() {return &_controller;}

private:

    Controller _controller;

    void loadPlugins(const boost::property_tree::ptree& pt);
};
} // namespace toffy
