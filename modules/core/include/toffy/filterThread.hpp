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

// P2-7: the threading primitives are the standard library's now. <list> is
// included on purpose -- std::list is used below and this file only compiled for
// years because <boost/thread.hpp> happened to pull it in
// (cleanup/findings/ownership.md, "A header include is commented out").
#include <atomic>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#include "toffy/filter.hpp"

namespace toffy
{
/**
 * @brief Runs one Filter in its own thread; a "lane" of a ParallelFilter.
 * @ingroup Core
 *
 * The lane owns a small pool of Frames and cycles them: dequeue() hands a
 * processed frame to the caller, enqueue() gives it back, and the worker thread
 * runs the filter on whatever is in the input queue. Everything the lane owns
 * is reachable from the two queues, which is why both are guarded by one mutex
 * (see the members).
 *
 * Threading (P2-7): std::thread, std::mutex, std::condition_variable and a
 * std::atomic run flag. They used to be boost::thread, boost::mutex and
 * boost::condition_variable, which differ in one way that matters here: at the
 * BOOST_THREAD_VERSION this tree builds with (2, the default), a boost::thread
 * *detaches itself* instead of failing when it is destroyed or move-assigned
 * while still joinable. std::thread calls std::terminate in both cases, so the
 * start()/stop()/destructor paths are explicit about the thread's state.
 */
class FilterThread
{
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
     * @brief Stops the worker, joins it if one was started, and frees the
     *        frames still in the queues.
     */
    virtual ~FilterThread();

    /**
     * A lane is an identity object: it owns a thread and two condition
     * variables. The copy constructor used to be declared and to share the
     * owned filter with the copy -- two lanes, one filter, one delete each --
     * and it could never have compiled, because neither std::thread nor
     * std::mutex is copyable. P2-7 says what the code has always meant.
     */
    FilterThread(const FilterThread&) = delete;
    FilterThread& operator=(const FilterThread&) = delete;

    /**
     * @brief init the FT with a number of pre-allocated frames
     * @param numFrames
     *
     * Additive, as it always was: calling init() twice queues twice. The frames
     * created here are the lane's and are deleted by ~FilterThread.
     */
    void init(int numFrames);

    /**
     * @brief start the thread
     *
     * Does nothing if the lane is already running, and collects a previous run
     * that has been stop()ed first. Both cases used to be a move-assignment over
     * a joinable thread, which detached the old worker instead of failing.
     */
    void start();

    /**
     * @brief stop the thread
     *
     * Signals the worker and releases anyone blocked in dequeue(). It does not
     * join: ~FilterThread does that, and a stop() that blocked until the worker
     * came back would make ParallelFilter::stop() block on a running lane.
     */
    void stop();

    /**
     * @brief synchronously retrieve an output frame
     * @return the next processed frame, or nullptr if the lane was stopped
     *         while the call was waiting
     */
    Frame* dequeue();

    /**
     * @brief enqueue a frame that is ready to be filled.
     *
     * The frame must be one this lane handed out: the lane owns every frame in
     * its queues and deletes the ones still there when it is destroyed.
     */
    void enqueue(Frame*);

   private:
    /**
     * @brief The filter this lane runs, shared with whoever else holds it
     * (normally the FilterBank it is pipelined into). P2-5.
     */
    FilterPtr f;

    /**
     * @brief The run flag: written by start()/stop(), read by the worker loop.
     *
     * std::atomic, not the plain bool it used to be: the loop condition was a
     * read of a non-volatile, non-atomic member written by another thread, which
     * the compiler may hoist out of the loop, so stop() could be ignored
     * (cleanup/findings/correctness.md, D). Every write happens under qMtx so
     * that the condition variables below cannot lose the wakeup.
     */
    std::atomic<bool> keepRunning{false};

    /// @brief The worker; joinable only between start() and the next join().
    std::thread theThread;

    /**
     * @brief Guards both queues and the run flag.
     *
     * There used to be one mutex per queue, and neither guarded the waits: each
     * wait built a *local* mutex, locked it and waited on the condition
     * variable, so the emptiness check and the wait were not atomic with respect
     * to the thread pushing the frame -- a frame pushed in that window was
     * notified to nobody. One mutex for the whole lane makes the invariant
     * statable, and the lane has one producer and one consumer, so there is
     * nothing to be gained by splitting it.
     */
    std::mutex qMtx;

    std::list<Frame*> inQ;  ///< Frames waiting to be run through the filter.
    std::list<Frame*> outQ;  ///< Processed frames waiting for their owner.

    std::condition_variable inCond;   ///< Signalled when inQ becomes non-empty.
    std::condition_variable outCond;  ///< Signalled when outQ becomes non-empty
                                      ///< or the lane stops.

    /**
     * @brief The worker body: wait for a frame, run the filter on it, publish
     * the result.
     */
    void loop();
};
}  // namespace toffy
