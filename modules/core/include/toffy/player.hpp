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
#pragma once

#include <boost/log/trivial.hpp>

#include <toffy/toffy_export.h>
#include <toffy/controller.hpp>
#include <toffy/filterfactory.hpp>


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
    Player(boost::log::trivial::severity_level severity,bool file);

    // defaults to debug, false
    Player();

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

    /** get the frame object of the underlying controller
     * @return the active frame
     */
    Frame& getFrame() { return _controller.getFrame(); }

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
