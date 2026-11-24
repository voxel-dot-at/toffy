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

// P2-7: the run thread is a std::thread. <boost/thread.hpp> is gone, and with
// it the requirement that anything including this header link Boost.Thread.
#include <atomic>
#include <memory>
#include <thread>

#include <toffy/filterbank.hpp>
#include <toffy/frame.hpp>
#include <toffy/common/plugins.hpp>  // P2-10: getLoadedFilters() returns the loader's handle list

namespace toffy {

/**
 * @brief Type name of the filter whose presence makes the controller pump the
 *        HighGUI event loop.
 *
 *  `Controller::loopFilters()` has to call `cv::waitKey()` while the pipeline
 *  runs, or any OpenCV window it spawned stops redrawing and the whole loop
 *  looks wedged. It decides whether to pump by counting filters of this type in
 *  the bank -- and until `P2-9` it got the string by including
 *  `<toffy/viewers/imageview.hpp>`, which made `modules/core` depend on the
 *  viewers module (and link it) for one identifier.
 *
 *  Core owns the name now. The cost of duplicating a string is drift, so
 *  `tests/test_layering.cpp` pins this against `ImageView::id_name` and fails if
 *  they ever disagree.
 *
 *  `inline constexpr` rather than `const`: a plain `const std::string` at
 *  namespace scope has internal linkage, so every translation unit that included
 *  it got its own copy and its own static initialiser. See also `P2-9` on
 *  `btaFrame.hpp`.
 */
inline constexpr const char* const guiViewerTypeName = "imageview";


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
class Controller {
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

    Controller();

    virtual ~Controller();

    /**
     * @brief Starts filtering in loop in a separated thread.
     * @return True on success, false if failed.
     */

    /**
     * @brief the one implementation behind forward/backward/stepForward/stepBackward
     *
     * P2-6. Those four were ~90% the same code -- look up the parallelFilters,
     * init and start them, set the state, spawn the loop, join -- which is four
     * places for the same thread-lifecycle bug to hide (finding A12 was one of
     * them, in stop()). The two real differences are the state and whether the
     * bank runs on the calling thread once or on a worker until stop().
     */
    bool run(state newState, bool singleStep);

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
    bool stepBackward();

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
     * @brief is this state one that the run loop should keep running for?
     *
     * P2-6. The run loop used to be `while (_state > Controller::IDLE)`, which
     * reads as "anything but idle" and is not: `CERROR = 0xff` is the largest
     * value in the enum, so an error state is greater than IDLE and the loop
     * spins forever instead of leaving. Nothing in toffy or either consumer ever
     * assigns CERROR, so this has never been observed -- but the enumerator is
     * public API, so the first person to use it would get a hang.
     *
     * Written as an explicit list rather than a range so that reordering the
     * enumerators cannot silently change what the loop does -- which is the same
     * class of accident the comparison just removed.
     */
    static bool isRunning(state s)
    {
        return s == START_FW || s == FORWARD || s == STEP_FW ||
               s == START_BW || s == BACKWARD;
    }

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

    /**
     * @brief the dlopen() handles of the plug-ins this process has loaded
     *
     * P2-10: these used to live in `Controller::_loads`. They now belong to
     * `toffy::commons::plugins`, which is the single loader and therefore the
     * single owner -- `FilterBank` had a second copy of the loop that threw its
     * handles away entirely. The return type is unchanged.
     */
    const std::vector<void *>& getLoadedFilters() const
    {
        return toffy::commons::plugins::loadedHandles();
    }

    /**
     * @brief the base and unique object container this Controller runs on
     *
     * Was the public data member `f`. P2-9 made it private: `Player` both wraps
     * `Controller` and handed it out via `getController()`, so the facade
     * protected nothing and any caller could reseat the frame under it.
     */
    Frame& getFrame() { return f; }

    /**
     * @brief read-only view of the frame, for const callers
     *
     * `Player::hasKey()` and friends are const and hold a const Controller, so
     * they need this; without it P2-9 would have had to make them non-const, which
     * is a wider change than the one being made.
     */
    const Frame& getFrame() const { return f; }

    /**
     * @brief the root FilterBank, or null before the pipeline is built
     *
     * Was the public data member `baseFilterBank` (P2-9). Prefer this accessor:
     * assigning to the member was a way to reseat the root bank without the
     * Controller's knowledge, and the ownership story below only holds if the
     * Controller is the only thing that ever assigns it.
     */
    const std::shared_ptr<FilterBank>& baseBank() const { return baseFilterBank; }

private:
    // P2-9: these two used to be public data members. `Player` wraps `Controller`
    // and also hands it out through `getController()`, so a public member meant the
    // facade protected nothing: any caller could reseat the root bank or the frame
    // underneath a running Controller. Making them private is a source change for
    // external code and not an ABI change -- the object layout is identical, which
    // is why this can land inside major 2. Measured before doing it: neither member
    // is referenced by toffy-oatpp or becom-knapp-wurthbox; the only external users
    // were Player (which has baseBank()/getFrame()) and toffy's own tests.

    /**
     * @brief keeps instance of the root FilterBank
     *
     * A shared_ptr since P2-5, because the Controller really is the owner: the
     * root bank is created here and nobody else is ever handed it. ~Controller
     * used to release it with FilterFactory::deleteFilter(baseFilterBank->id()),
     * i.e. by asking a process-wide registry to `delete` an object this object
     * owned -- which released it only while the registry still knew the name,
     * and destroyed it while any other holder of the pointer was still using
     * it. Letting the FilterPtr go is now the teardown, and the bank's own
     * children go with it.
     *
     * `->`, `== nullptr` and `.get()` all behave as they did with the raw
     * member. Code that assigned the member to a `FilterBank*` needs `.get()`.
     */
    std::shared_ptr<FilterBank> baseFilterBank;
    Frame f; ///< base and unique object container

    /**
     * @brief thread to run the toffy filtering
     *
     * Joinable between a forward()/backward() that started it and the next
     * stop() -- or, when loopFilters() ended the run itself by putting the state
     * back to IDLE, between that and whatever comes next. The run methods join it
     * before starting a new one: assigning over a joinable std::thread calls
     * std::terminate(), where boost::thread at BOOST_THREAD_VERSION 2 silently
     * detached the old worker.
     */
    std::thread _thread;
    /**
     * @brief running state of toffy
     *
     * `std::atomic` since P2-6. It is written by `stop()` and `run()` on the
     * caller's thread and read every iteration by `loopFilters()` on the worker,
     * with no lock anywhere between them -- it used to be a plain `state`, and
     * ThreadSanitizer reported the pair (controller.cpp `stop()` vs `loopFilters()`)
     * as soon as anything exercised them, which nothing did until the P2-6 tests.
     * A non-atomic flag here is not theoretical: the compiler may hoist the read
     * out of the loop, so `stop()` is ignored and the worker never exits.
     *
     * Layout-compatible with the plain enum (same size and alignment), so this is
     * not an ABI change -- but it is a header change, so everything including
     * controller.hpp must be recompiled.
     */
    std::atomic<state> _state;
    // P2-10: `std::vector<void*> _loads` used to sit here. The plug-in handles
    // now have one owner, toffy::commons::plugins -- see getLoadedFilters().

    void loopFilters();
    void loopFiltersOnce();
};
} // namespace toffy

#endif // __toffy_CONTROLLER_HPP__
