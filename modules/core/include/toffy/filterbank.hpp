#pragma once

#include <vector>
#include "toffy/filterfactory.hpp"
#include <boost/container/flat_set.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#ifdef MSVC
#define DLLExport __declspec( dllexport )
#define RAWFILE ".rw"
#else
#define DLLExport /**/
#define RAWFILE ".r"
#endif

namespace toffy {

/**
 * @brief Container of filters
 * @ingroup Core
 *
 * The FilterBank is a special Filter that contains filters. The reason to be
 * defined as a filter is that the can be so chained. In this way we can have
 * filter organized in a tree structure.
 *
 * A FilterBank is a secuencially process line. the filters included on it are
 * execute one after the other.
 *
 * It also the start point for toffy. A base FilterBank is defined in Player and
 * is the base for creating the execution structure.
 *
 * It is able to read config files and load all the Filters and information.
 *
 * For a description of the load config files:
 * @see FilterBank::loadFileConfig()
 *
 */
class TOFFY_EXPORT FilterBank: public Filter
{
public:

    static const std::string id_name; ///< Const with the filter type

    /**
     * @brief FilterBank
     *
     */
    FilterBank() : Filter(id_name, _filter_counter), ready(0)
    {
	ff = FilterFactory::getInstance();
	this->bank(this);
    }

    /**
     * @brief Constructor for derived classes
     * @param type
     * @param counter
     */
    FilterBank(std::string type, std::size_t counter) : Filter(type, counter), ready(0)
    {
	ff = FilterFactory::getInstance();
	this->bank(this);
    }

    /**
     * @brief ~FilterBank
     */
    virtual ~FilterBank();

    /**
     * @brief Adds a Filter to the filterBank
     * @param f Filter
     */
    void add(Filter* f);

    /**
     * @brief Adds a filter in a determined position of the FilterBank
     * @param pos int
     * @param f Filter
     */
    void insert(int pos, Filter* f) {
	std::vector<Filter*>::iterator it = _pipe.begin();
	_pipe.insert(it+pos, f);
    }

    /**
     * @brief Remove all filter from the FilterBank
     */
    void clearBank();

    /**
     * @brief const getFilter by name
     * @param name
     * @return Filter
     *
     * @todo Use getPosition and getfilter(int) to implement it local
     * @deprecated We want the getFilter to be local to the FilterBank
     *
     */
    const Filter * getFilter(const std::string& name) const;

    /**
     * @brief const getFilter by position
     * @param i
     * @return Filter
     *
     */
    const Filter * getFilter(int i) const;

    /**
     * @brief getFilter by name
     * @param name
     * @return Filter
     *
     * @todo Use getPosition and getfilter(int) to implement it local
     * @deprecated We want the getFilter to be local to the FilterBank
     *
     */
    Filter * getFilter(const std::string& name);

    /**
     * @brief getFilter by position
     * @param i
     * @return Filter
     */
    Filter * getFilter(int i);

    /**
     * @brief Gets all filter in a FilterBank by type
     * @param type
     * @param vec
     * @return std::vector<Filter *>
     *
     * @todo Move to FilterFactory and reimplement localy
     */
    int getFiltersByType(const std::string& type,
					 std::vector<Filter *> &vec);

    /**
     * @brief Counts all filter in a FilterBank by type
     * @param type
     * @return std::vector<Filter *>
     *
     * @todo Move to FilterFactory and reimplement localy
     */
    int countFiltersByType(const std::string& type);

    /**
     * @brief Removes a filter from the FilterBank by name
     * @param name
     * @return Positive on success, negative or 0 in failed
     */
    int remove(std::string name);

    /**
     * @brief Removes a filter from the FilterBank by position
     * @param i
     * @return Positive on success, negative or 0 in failed
     */
    int remove(size_t i);

    /**
     * @brief Get the position of a filter in the filter bank
     * @param name
     * @return Positive position on success, negative if not found
     */
    size_t findPos(std::string name);

    /**
     * @brief Get ne number of filter in the FilterBank
     * @return size_t
     */
    size_t size() const {return _pipe.size();}

    /**
     * @brief processEvent
     * @param e
     *
     * @todo Document
     */
    virtual void processEvent(Event &e);

    /**
     * @brief Runs all defined filters and accumulate result for external api
     * @return true if processing worked, false on error
     */
    virtual bool filter(const Frame& in, Frame& out);

    /**
     * @brief loadFileConfig
     * @param configFile
     * @return Positive on success, negative or 0 if failled
     *
     * Load the XML file into the property tree. If reading fails
     * (cannot open file, parse error), an exception is thrown.
     * Checks the root node
     *
     * The loading filter secuence is as follows:
     *
     * - loadFileConfig is the first method to be call (from the base
     * FilterBank in the Player)
     * It loads the file, check the root node \<toffy> and try to load globals.
     *
     * - loadConfig(const std::string&, const boost::property_tree::ptree&) is
     * called which tries to load all the filter at the root level by calling
     * loadConfig(const std::string& configFile,
     * 			   const ptree::const_iterator& begin,
     * 			   const ptree::const_iterator& end)
     *
     * - The last method iterate over the all the nodes at that level and
     * creates the filters calling handleConfigItem(
     *	    const std::string& configFile, const ptree::const_iterator& it)
     *
     * - handleConfigItem check for special filter and create normal ones with
     * instantiateFilter(ptree::const_iterator& it)
     *
     * - The filter is created through the FilterFactory
     *
     */
    virtual int loadFileConfig(const std::string& configFile);

    //virtual int loadConfig(const boost::property_tree::ptree& pt);

    /**
     * @brief Returns the FilterBank that contains this FilterBank
     * @return Filter in exist or NULL if bse FilterBank
     *
     * If the Object does not have a Father FilterBank, that means that it is
     * the base FilterBank and the funtion returns NULL
     *
     */
    FilterBank* getBaseFilterbank();

    /**
     * @brief Get a node in FilterBank local globals or a ptree to them
     * @param node
     * @return boost::property_tree::ptree
     *
     * Gets the node selected by the parameter in the local globals if exists.
     * In parameter empty, or node does not exists, return the ptree to the
     * local to FilterBank globals
     *
     */
    const boost::property_tree::ptree& getGlobals(std::string node  = "");

    /**
     * @brief Recursive version
     * @param confFile
     * @param pt
     * @return Positive on success, negative or 0 if failed
     */
     virtual int loadConfig(const boost::property_tree::ptree& pt, const std::string& confFile = "");


    virtual boost::property_tree::ptree getConfig() const;

    /**
     * @brief wait
     */
    virtual void wait() { ready.wait(); }

    /**
     * @brief try_wait
     * @return
     */
    virtual bool try_wait() { return ready.try_wait(); }

    virtual void init();
    virtual void start();
    virtual void stop();

protected:

    /**
     * @brief Load a set of Filter in the FilterBank
     * @param confFile
     * @param begin
     * @param end
     * @return Positive on success, negative or 0 if failed
     *
     * Load all the filter contained in a ptree that commes from a
     * \<filterBank> node or the base node \<toffy>
     *
     */
    virtual int loadConfig(const std::string& confFile,
			   const boost::property_tree::ptree::const_iterator& begin,
			   const boost::property_tree::ptree::const_iterator& end);

    /**
     * @brief Loads a single Filter from the ptree
     * @param confFile
     * @param it
     * @return Positive on success, negative or 0 if failed
     *
     * Check every element of the config file and creates the filter declared
     * on it.
     *
     * XML comment nodes are ignore as well as the globals as we want them
     * loaded before any filter is created.
     *
     * Special nodes:
     * \<filterGroup>: loads filters from a file in a separated FilterBank
     *
     *
     * @todo Add node for loading in a separated FilterBank in the same file
     *
     */
    virtual int handleConfigItem(const std::string& confFile,
			   const boost::property_tree::ptree::const_iterator& it);

    /**
     * @brief Creates a new filter instance based on the iterator contents
     * @param it
     * @return Filter
     *
     * it calls loadConfig on the filter and registers it with the
     * filterfactory.
     *
     */
    virtual Filter* instantiateFilter(
	    const boost::property_tree::ptree::const_iterator& it);

    int loadPlugins(const boost::property_tree::ptree& pt);

private:

    FilterFactory *ff; ///< Pointer to the FilterFactory
    std::vector<Filter*> _pipe; ///< Filter container

    boost::property_tree::ptree _globalConfig; ///< Ptree to local to FilterBank globals

    boost::interprocess::interprocess_semaphore ready; ///< TODO

    static std::size_t _filter_counter; ///< Internal Filter counter

    /**
     * @brief Overrides the Filter loadGlobals to save the local FilterBank globals
     * @param pt
     * @return Positive on success, negative or 0 if failed
     */
    virtual int loadGlobals(const boost::property_tree::ptree& pt);
};

}
