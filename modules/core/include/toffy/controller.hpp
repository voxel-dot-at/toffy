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
#ifndef __toffy_CONTROLLER_HPP__
#define __toffy_CONTROLLER_HPP__

#include <toffy/filterbank.hpp>
#include <boost/thread.hpp>
#include <toffy/frame.hpp>

namespace toffy {


/**
 * @brief toffy controller
 * @ingroup Core
 *
 * This class provides access to toffy actions and objects. It is an interface
 * for every ui that allows to control toffy.
 * It contains the base filterBank and the Frame container with which is defined
 * and toffy instance.
 *
 * @todo Forse constructor with Player instance. Player is the basic ui and
 * should be present, among others.
 *
 * @todo If we want methods to simple run filter without changing the backward
 * flag, we should add them, to allow users to set this flag differently for +
 * each capturer. Now it can be done because we set and remove a in-Frame data
 * flag.
 *
 *
 */
class TOFFY_EXPORT Controller {
public:

    /**
     * @brief Possible states of toffy as whole.
     */
    enum state {
        IDLE = 0,
        START_FW,
        FORWARD,
        STEP_FW,
        START_BW,
        BACKWARD,
        CERROR = 0xff
    };

    FilterBank * baseFilterBank; ///< keeps instance of the root FilterBank
    Frame f; ///< base and unique object container

    Controller();

    virtual ~Controller();

    /**
     * @brief Starts filtering in loop in a separated thread.
     * @return True on success, false if failed.
     */

    bool forward();
    /**
     * @brief Same as forward but set the backward flag for all capturers.
     * @return True on success, false if failed.
     *
     * @todo Now this flag is in the frame. Move to set the flag in
     * CapturerFilter. Create and use getAllCaptureres and set flag.
     */
    bool backward();

    /**
     * @brief Does a single iteration for all instanciate filters.
     * @return True on success, false if failed.
     */
    bool stepForward();

    /**
     * @brief Same as stepForward but setting the frame backward flag
     * @return True on success, false if failed.
     *
     * @todo
     */
    bool stedBackward();

    /**
     * @brief Just stop filtering if running in thread
     * @return True on success, false if failed.
     */
    bool stop();

    /**
     * @brief Getter for the toffy state
     * @return The toffy running state
     */
    Controller::state getState() {return _state;}

    /**
     * @brief Allows saving the filters public runtime configuration values in
     * a toffy configFile.
     * @param fileName Path and name of the file to save.
     *
     * The runtime configuration file is usefull for importing exporting
     * configuration when doing changes in files. This file will not complete to
     * load all the filters and config as the configFile
     * (startUp configs, global, etc data are missing). But will overwrite it
     * when the config is loaded.
     *
     */
    void saveRunConfig(std::string fileName);

    /**
     * @brief Loads a runtime config file and does an updateConfig for each root
     * node (filter).
     *
     * @param configFile Path and name of the file.
     * @return Positive on success, false if failed.
     */
    int loadRuntimeConfig(const std::string &configFile);

    void loadPlugins(const boost::property_tree::ptree& pt);

    void loadPlugin(std::string lib);

    int loadConfigFile(const std::string &configFile);

    int loadConfig(const boost::property_tree::ptree& pt);

    std::vector<void *> const getLoadedFilters() const {return _loads;}

private:
    boost::thread _thread; ///< thread to run the toffy filtering
    state _state; ///< running state of toffy.
    std::vector<void *> _loads;

    void loopFilters();
    void loopFiltersOnce();
};
} // namespace toffy

#endif // __toffy_CONTROLLER_HPP__
