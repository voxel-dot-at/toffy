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
//#include <string>
//#include <list>

// <memory> for the FilterPtr member, and <list> for the two queues: the list
// include has been commented out since forever and the file compiles only
// because boost/thread pulls it in. Restoring it is P2-7's, not this PR's;
// <memory> is here because this PR introduces the FilterPtr.
#include <memory>
#include <utility>

#include <boost/thread/thread.hpp>

#include "toffy/filter.hpp"

namespace toffy
{
/**
     * @brief Class FilterThread enables execution of filters in a thread.
     * @ingroup Core
     *
     * Executes all Filter presents in a FilterBank using a separated thread.
     * The FilterBank it self is execute always sequentially
     *
     */
class FilterThread {
public:
    /**
	 * @brief Constructor: the FilterThread shares ownership of the filter.
	 * @param filter
	 *
	 * The doc comment used to say the filter "is from then owned by FT and will
	 * be deleted in the destructor", and the destructor did `delete f` -- while
	 * the FilterBank that had created the same filter also held it in its
	 * pipeline and released it through the factory. Two owners, and
	 * ~ParallelFilter then ran the thread's delete before the bank's
	 * (cleanup/findings/ownership.md, C1).
	 *
	 * It now holds a FilterPtr: the lane keeps the filter alive for as long as
	 * the thread exists, which is the property the old code wanted, and the
	 * object dies when the last of the holders lets go. A lane whose filter is
	 * in no bank (ParallelFilter's `new FilterBank()` threads) still gets it
	 * destroyed, because the FilterThread is then the only owner.
	 *
	 * explicit: an implicit Filter* -> FilterPtr conversion here would hand this
	 * class ownership of a pointer it did not create, which is the bug class
	 * P2-5 is removing.
	 */
    explicit FilterThread(FilterPtr filter) : f(std::move(filter)) {}

    /**
	 * @brief ~FilterThread
	 */
    virtual ~FilterThread();

    /**
	 * @brief FilterThread
	 * @param ft
	 *
	 * Still declared, still uncompilable (boost::thread and boost::mutex are not
	 * copyable), and still the wrong idea: it copied the owned filter, giving
	 * two FilterThreads the same one. Deleting it is P2-7's; what changed here is
	 * that the copy it describes would now share instead of double-owning.
	 */
    FilterThread( const FilterThread& ft): f(ft.f) {}

    /**
	 * @brief init the FT with a number of pre-allocated frames
	 * @param numFrames
	 */
    void init(int numFrames);

    /**
	 * @brief start the thread
	 */
    void start();

    /**
	 * @brief stop the thread
	 */
    void stop();

    /**
	 * @brief synchronously retrieve an output frame
	 * @return
	 */
    Frame* dequeue();

    /**
	 * @brief enqueue a frame that is ready to be filled.
	 */
    void enqueue(Frame*);

private:
    /**
     * @brief The filter this lane runs, shared with whoever else holds it
     * (normally the FilterBank it is pipelined into). P2-5.
     */
    FilterPtr f;

    bool keepRunning; ///< @todo document

    boost::thread theThread; ///< @todo document

    std::list<Frame*> inQ; ///< @todo document
    boost::mutex inMtx; ///< @todo document
    boost::condition_variable inCond; ///< @todo document

    std::list<Frame*> outQ; ///< @todo document
    boost::mutex outMtx; ///< @todo document
    boost::condition_variable outCond; ///< @todo document

    /**
     * @brief loop
     * @todo document
     */
    void loop();
};
}
