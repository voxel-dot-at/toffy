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

    /**
     * @brief Container for all parallel filter threads.
     *
     * unique_ptr, not raw pointers (P2-7): the bank owns the lanes, and the
     * vector used to be emptied by a loop of `delete lanes[i]` -- the last raw
     * delete in modules/core (cleanup/counters.md counter 13). ~ParallelFilter
     * still calls stop() on every lane before they are destroyed.
     */
    std::vector<std::unique_ptr<FilterThread>> lanes;

    /**
     * @brief To synchronize all Filter outputs; null when the config has no
     * barrier node.
     *
     * The `= nullptr` is not decoration: the constructor never initialised this,
     * so a ParallelFilter configured without a <barrier> read an indeterminate
     * pointer in filter()'s `if (mux)` and could call through it. Same defect
     * class as P0-5 (uninitialised members), found while reading for P2-7.
     */
    Mux* mux = nullptr;
};
}
