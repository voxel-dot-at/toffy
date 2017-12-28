#pragma once

#include <vector>
#include <boost/container/flat_set.hpp>
#include <boost/property_tree/ptree.hpp>

#include "toffy/filterbank.hpp"
#include "toffy/filterThread.hpp"


namespace toffy
{
class Mux;

/**
 * @brief ParallelFilter treats a collection of filters as a (parallel) set of
 * tasks, collecting the output frames for a subsequent (array) filter.
 * @ingroup Core
 */
class ParallelFilter : public FilterBank
{
public:

    static const std::string id_name; ///< Const with the filter type

    ParallelFilter() : FilterBank(id_name, _filter_counter) {}

    virtual ~ParallelFilter();

    /**
     * @brief Run all defined filters and accumulate result for external api
     *
     * @return true if processing worked, false on error
     */
    virtual bool filter(const Frame& in, Frame& out);


    /**
     * @brief initialize the threads and supply them with input frames
     */
    virtual void init();

    /**
     * @brief start all threads
     */
    virtual void start();

    /**
     * @brief stop and join all threads
     */
    virtual void stop();

protected:
    /**
     * @brief Handles the parallel filters elements
     * @param confFile
     * @param it
     * @return Positive on success, Negative or 0 if failed
     *
     *
     * Reads through a configFile node and checks the known parallel filter
     * nodes:
     * - parallelFilter, thread, filterGroup, and barrier.
     *
     *  It creates a list of filterBanks that will be execute in parallel.
     *
     */
    virtual int handleConfigItem(const std::string& confFile,
				 const boost::property_tree::ptree::const_iterator& it);

private:
    static std::size_t _filter_counter; ///< Internal Filter counter
    std::vector<FilterThread*> lanes; ///< Container for all parallel filter threads
    Mux* mux; ///< To synchronize all Filter outputs
};
}
