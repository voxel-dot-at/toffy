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

#include <toffy/toffy_export.h>
#include <toffy/toffy_config.h>

#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <boost/log/trivial.hpp>

#include <toffy/frame.hpp>

#ifdef MSVC
#define DLLExport __declspec( dllexport )
#else
#define DLLExport /**/
#endif

/**
 */
#ifdef OpenCV_VERSION_MAJOR
#  ifndef OCV_MAJOR_VERSION
#    define OCV_MAJOR_VERSION OpenCV_VERSION_MAJOR
#  endif
#endif


/** @defgroup Core Core
 *
 * Core module
 *
 */

/** @defgroup Filters Filters
 *
 * Filters module
 *
 */

/** @defgroup Viewers Viewers
 *
 * Viewers module
 *
 */

namespace toffy {

class Event;
class Filter;

/**
 * @brief Filter running state definitions
 */
enum filterState {
    filterLoaded,
    filterIdle,
    filterRunning,
    filterPaused,
    filterError};

/**
 * @brief The FilterListener class
 */
class FilterListener {
public:
    /**
     * @brief stateChanged
     * @param source
     * @param state
     */
    virtual void stateChanged(Filter& source, filterState state) = 0;
};

/**
 * @brief The Port class
 */
class Port { // meta data to be done...
public:
    /**
     * @brief The portDir enum
     */
    enum portDir { portIn, portOut };

    portDir dir; ///< dir
    std::string name; ///< name
};

/**
 * @brief Base clase filter
 * @ingroup Core
 *
 * This class define how is a filter and is capabilities.
 *
 */
class TOFFY_EXPORT Filter {

    std::string _id, ///< Internal id of the single instance of the filter
    _name, ///< id alias @deprecated Is the same as id
    _type; ///< Type id of the filter

    Filter *_bank; ///< reference to the filter bank whre the filter resides
    //static std::size_t _filter_counter;

    /*
     * From boost trivial log
     * //! Trivial severity levels
    enum severity_level
    {
        trace,
        debug,
        info,
        warning,
        error,
        fatal
    };
     * @todo Find a better way to individually set log level.
     * changing severity filter affects everithing.
     */
    boost::log::trivial::severity_level _log_lvl;

public:

    bool dbg, ///< flag for activating debug options (images view, log, etc)
    update; ///< Flag for indicating config updated

    /**
     * @brief Filter
     */
    Filter();

    /**
     * @brief ~Filter
     */
    virtual ~Filter();

    boost::log::trivial::severity_level logLvl() const {return _log_lvl;}

    /**
     * @brief getCounter
     * @return unsigned int Actuall index of filters
     */
    unsigned int getCounter() const;

    /**
     * @brief Getter type
     * @return std::string The type of the filter
     */
    const std::string& type() const { return _type;}

    /**
     * @brief Getter name
     * @return std::string The name of the filter
     *
     * Name of the filter is by default a combination of type+counter
     * at least the users indicates a name explicitly in the config
     */
    const std::string& name() const { return _name;}

    /**
     * @brief Setter name
     * @param n
     *
     * Name of the filter is by default a combination of type+counter
     * at least the users indicates a name explicitly in the config
     */
    void name(std::string n) {_name = n;}

    /** perform a filter operation on in, sending result to out
     * both Mat's may refer to the same memory region.
     *
     * @return true if processing worked, false on error
     */
    virtual bool filter(const Frame& /*in*/, Frame& /*out*/) const { return false; }

    /**
     * @brief perform a filter operation on in, sending result to out
     * both Mat's may refer to the same memory region.
     *
     * @param in
     * @param out
     * @return true if processing worked, false on error
     *
     * The default behavior of this non-const variant is to call
     * the const filter method defined above.
     *
     */
    virtual bool filter(const Frame& in, Frame& out) {
        const Filter& constF = *this;
        return constF.filter(in, out) ;
    }

    /**
     * @brief id
     * @return
     */
    std::string id() const {return _id;}

    /**
     * @brief loadConfig
     * @param pt ptree
     * @return Positive on success, negative or 0 if failled
     *
     * Loads the configuration for the first time. It checks if the \<globals>
     * node is present and update the configuration from them. Then calls
     * updateConfig to load the rest of the configuration. The local
     * configuration overrides the global.
     *
     * The derived filter can overwrite this methos to implement some extra
     * action at filter creation time (like autoconnect to a camera).
     * If so, the parent loadConfig should be called at least we are handling
     * a special filter like the FilterBank.
     *
     */
    virtual int loadConfig(const boost::property_tree::ptree& pt);

    /**
     * @brief Put all public config values in a ptree
     * @return ptree
     *
     * Its use to get runtime the filter configuration.
     */
    virtual boost::property_tree::ptree getConfig() const;

    /**
     * @brief Udate the internal configuration of the filter
     * @param pt ptree
     *
     * Read a ptree and set the filter values.
     * The base filter initializes common variables like the log level.
     *
     * Each filter should overwrite this method and add the filter specific
     * variables to be read.
     *
     * This method does not care from where the configuration is comming:
     *  first loadConfig, loadGlobals or external like UI.
     * It just update the config values.
     *
     */
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    /**
     * @brief Look for global nodes that contain information for the filter
     * @param pt ptree to a configuration
     *
     * Determines if the filter wants to load values from globals checking the
     * \<globals> node inside the filter configuration.
     *
     * Looks if in the globals node (if exists) threre are values which are
     * valuable for the filter. Normally this values are use also in other
     * filters and to avoid write them several time in the config, we can set
     * them to the globals.
     *
     */
    void loadGlobals(const boost::property_tree::ptree &pt);

    /**
     * @brief Getter filterBank
     * @return FilterBank
     */
    Filter * bank() const {return _bank;}

    /**
     * @brief Setter FilterBank
     * @param bank FilterBank where the filter resides
     */
    void bank(Filter * bank) {_bank=bank;}

    /**
     * @brief processEvent
     * @param e Data if the event to be runned
     *
     * Every filter could define in this method a set of actions that could
     * be executed by call from other filters.
     *
     * The events are processed outside of the FilterBank sequential mode.
     * Every filter must discribe his availabled events.
     *
     */
    virtual void processEvent(Event &e);


protected:

    /**
     * @brief Base constructor for any filter
     * @param type [in] Id of the filter
     * @param counter Int value to be append to the name of the filter
     */
    Filter(std::string type, std::size_t counter = 0);

    /*!
     *** state handling ************************
     *  \addtogroup State_handling
     *  @{
     */
private:
    filterState state; ///< Filter state
    std::vector<FilterListener*> listeners; ///< listeners
    std::string errMsg; ///< error


    /**
     * @brief loadFileConfig
     * @param configFile
     * @return Positive on success, negative or 0 if failled
     *
     * Load the XML file into the property tree. If reading fails
     * (cannot open file, parse error), an exception is thrown.
     *
     * @deprecated We dont want a filter reading files, only the filterbank
     *
     */
    virtual int loadFileConfig(const std::string& configFile);

public:
    /**
     * @brief getState
     * @return
     *
     */
    virtual filterState getState() { return state; }

    /**
     * @brief addListener
     * @param l
     */
    void addListener(FilterListener* l) { listeners.push_back(l); }

    /**
     * @brief removeListener
     * @param l
     */
    inline void removeListener(const FilterListener* l);

    /**
     * @brief Optional step to pre-allocate resources, if not done in the
     * config updates, reset the state to idle
     *
     */
    inline virtual void init();

    /**
     * @brief start the filter -- active components will connect to
     * cameras etc.
     */
    inline virtual void start();

    /**
     * @brief stop processing
     */
    inline virtual void stop();

    /**
     * @brief enter error state, pass an error string
     */
    inline virtual void error(const std::string& errMsg);

    /**
     * @brief getErrorMsg
     * @return std::string&
     */
    const std::string& getErrorMsg() { return errMsg; }

    /**
     * @brief Sets the boost log filter severity
     */
    void setLoggingLvl();


protected:
    /**
     * @brief set the filter state and notify listeners on a change.
     */
    virtual void setState(filterState state);

    /*! @} End Group State_handling*/
};



void Filter::init() {
    errMsg = "";
    setState(filterIdle);
}

void Filter::start() {
    setState(filterRunning);
}

void Filter::stop() {
    setState(filterIdle);
}

void Filter::error(const std::string& e) {
    errMsg = e;
    setState(filterError);
}
}

