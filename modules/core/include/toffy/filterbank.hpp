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

#include <memory>
#include <optional>
#include <vector>

#include <boost/container/flat_set.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include "toffy/filterfactory.hpp"

// RAWFILE used to be defined here, in capturerFilter.hpp and in BtaWrapper.hpp --
// three public headers, each with its own MSVC #ifdef, so any translation unit that
// transitively included two of them risked a redefinition. Its only consumer is
// bta.cpp, so it now lives once, in toffy/bta/BtaWrapper.hpp. DLLExport also went:
// TOFFY_EXPORT is the real export macro.

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
 * Ownership (P2-5): the pipeline is a vector of FilterPtr and the bank is what
 * keeps its filters alive. add() takes shared ownership of the filter it is
 * given; remove(), clearBank() and the destructor give it up, and the filter is
 * destroyed when the last owner lets go -- which is this bank unless somebody
 * else took a FilterPtr first. Nothing calls delete on a Filter.
 *
 * That replaces the previous arrangement, where _pipe held raw pointers, the
 * FilterFactory registry held raw pointers to the same objects, and the bank's
 * teardown destroyed them by looking each one up *by name* in the factory. A
 * filter that had been renamed was therefore never released (leak), and a
 * filter that was in two banks was destroyed twice (double free) -- see
 * cleanup/findings/ownership.md (C1, A17).
 *
 * It is able to read config files and load all the Filters and information.
 *
 * For a description of the load config files:
 * @see FilterBank::loadFileConfig()
 *
 */
class TOFFY_EXPORT FilterBank : public Filter
{
   public:
    static const std::string id_name;  ///< Const with the filter type

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
    FilterBank(std::string type, std::size_t counter)
        : Filter(type, counter), ready(0)
    {
        ff = FilterFactory::getInstance();
        this->bank(this);
    }

    /**
     * @brief ~FilterBank
     */
    virtual ~FilterBank();

    /**
     * @brief Adds a Filter to the filterBank, sharing ownership of it
     * @param f FilterPtr; the bank takes a reference to whatever it points at
     *
     * The bank keeps a strong reference, so the filter outlives the caller's
     * own FilterPtr. An empty FilterPtr is refused with a warning instead of
     * being pipelined: a null in _pipe is a crash several frames later, in a
     * place with no connection to the call that put it there.
     *
     * The raw-pointer overload is gone on purpose, and `add(new X())` does not
     * compile either: shared_ptr's constructor from a raw pointer is `explicit`
     * (LWG 2904, and libstdc++ enforces it in every mode this project builds),
     * so there is no implicit route from a bare `Filter*` into the pipeline.
     * That is the point. The old signature accepted a borrowed pointer and
     * silently took a second, independent claim on the object -- which is how a
     * filter in two banks came to be deleted by the first one. Callers now have
     * to hand over a FilterPtr, so the ownership is visible at the call site:
     * `bank.add(factory->createFilter(type))`, or `add(std::make_shared<X>())`
     * for the rare case of a filter built outside the factory.
     */
    void add(FilterPtr f);

    /**
     * @brief Adds a filter in a determined position of the FilterBank
     * @param pos int, 0..size() inclusive (size() appends)
     * @param f FilterPtr, see add() for the ownership rules
     * @throws std::out_of_range if pos is negative or greater than size()
     *
     * The bounds check is new with P2-5. The old body was
     * `_pipe.insert(_pipe.begin() + pos, f)` with no check at all, so a
     * negative or oversized pos was undefined behaviour rather than an error.
     * An empty FilterPtr is refused the same way add() refuses it, and the
     * filter the rejected call created is destroyed on the way out.
     */
    void insert(int pos, FilterPtr f);

    /**
     * @brief Remove all filter from the FilterBank
     *
     * Drops the bank's reference to every filter; a filter is destroyed if the
     * bank was its last owner. No factory round-trip and no name lookup, which
     * is what made the old version leak renamed filters and double-free filters
     * that were in two banks (P2-5).
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
    const Filter* getFilter(const std::string& name) const;

    /**
     * @brief const getFilter by position
     * @param i
     * @return Filter
     *
     * Non-owning observer into _pipe; valid while this bank holds the filter.
     * Throws std::out_of_range if i is not a position in the pipeline.
     */
    const Filter* getFilter(int i) const;

    /**
     * @brief getFilter by name
     * @param name
     * @return Filter
     *
     * @todo Use getPosition and getfilter(int) to implement it local
     * @deprecated We want the getFilter to be local to the FilterBank
     *
     */
    Filter* getFilter(const std::string& name);

    /**
     * @brief getFilter by position
     * @param i
     * @return Filter
     *
     * Non-owning observer into _pipe; valid while this bank holds the filter.
     * Throws std::out_of_range if i is not a position in the pipeline.
     */
    Filter* getFilter(int i);

    /**
     * @brief Gets all filter in a FilterBank by type
     * @param type
     * @param vec appended-to; holds non-owning observers, not FilterPtr, so the
     *        caller cannot accidentally keep a filter alive through a pipeline
     *        it does not own
     * @return std::vector<Filter *>
     *
     * @todo Move to FilterFactory and reimplement localy
     */
    int getFiltersByType(const std::string& type, std::vector<Filter*>& vec);

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
     * @param name Name of the filter to remove
     * @return 1 if a filter was removed, 0 if no filter has that name
     *
     * Removes it from *this* pipeline; the filter itself survives as long as
     * somebody else holds a FilterPtr for it. Since P2-5 the caller can keep
     * one, and the removed filter is not destroyed under its feet.
     */
    int remove(const std::string& name);

    /**
     * @brief Removes a filter from the FilterBank by position
     * @param i Index of the filter to remove
     * @return 1 if a filter was removed, 0 if i is out of range
     */
    int remove(size_t i);

    /**
     * @brief Get the position of a filter in the filter bank
     * @param name Name of the filter to look up
     * @return Index of the filter in the pipeline, or std::nullopt if no
     * filter has that name
     */
    std::optional<int> findPos(const std::string& name) const;

    /**
     * @brief Get ne number of filter in the FilterBank
     * @return size_t
     */
    size_t size() const { return _pipe.size(); }

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
     * The loading filter sequence is as follows:
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

    // virtual int loadConfig(const boost::property_tree::ptree& pt);

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
    const boost::property_tree::ptree& getGlobals(std::string node = "");

    /**
     * @brief Recursive version
     * @param confFile
     * @param pt
     * @return Positive on success, negative or 0 if failed
     */
    virtual int loadConfig(const boost::property_tree::ptree& pt,
                           const std::string& confFile);

    virtual int loadConfig(const boost::property_tree::ptree& pt)
    {
        return loadConfig(pt, "");
    };

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
    virtual int loadConfig(
        const std::string& confFile,
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
    virtual int handleConfigItem(
        const std::string& confFile,
        const boost::property_tree::ptree::const_iterator& it);

    /**
     * @brief Creates a new filter instance based on the iterator contents
     * @param it
     * @return FilterPtr, empty if the type is unknown
     *
     * it calls loadConfig on the filter and registers it with the
     * filterfactory.
     *
     * Returns a FilterPtr since P2-5: the caller has to be able to hand the
     * ownership on to add(), and returning a raw pointer here meant the caller
     * had to arrange ownership of something it did not create. Derived classes
     * that override this (and there are none in this repository) change shape
     * with it.
     */
    virtual FilterPtr instantiateFilter(
        const boost::property_tree::ptree::const_iterator& it);

    int loadPlugins(const boost::property_tree::ptree& pt);

   private:
    FilterFactory* ff = nullptr;  ///< Pointer to the FilterFactory (a singleton,
                                  ///< so a non-owning observer)
    /**
     * @brief Filter container; the bank owns what is in it (P2-5).
     *
     * Shared pointers, not raw ones: the pipeline used to be a vector of
     * borrowed pointers while the FilterFactory registry borrowed the same
     * objects and ~FilterBank destroyed them through the factory by name. Two
     * borrowers and no owner is how a renamed filter leaked and a filter in two
     * banks was freed twice.
     */
    std::vector<FilterPtr> _pipe;

    boost::property_tree::ptree
        _globalConfig;  ///< Ptree to local to FilterBank globals

    boost::interprocess::interprocess_semaphore ready;  ///< TODO

    static std::size_t _filter_counter;  ///< Internal Filter counter

    /**
     * @brief Overrides the Filter loadGlobals to save the local FilterBank
     * globals
     * @param pt
     * @return Positive on success, negative or 0 if failed
     */
    virtual int loadGlobals(const boost::property_tree::ptree& pt);
};

}  // namespace toffy
