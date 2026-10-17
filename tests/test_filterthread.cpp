/*
   Copyright 2026 Simon Vogl <svogl@voxel.at>

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

// Threading (P2-7): FilterThread, and the ParallelFilter lanes that own one.
//
// Why this file exists rather than a one-line type substitution. The tree used
// boost::thread, boost::mutex and boost::condition_variable, and at the
// BOOST_THREAD_VERSION this build selects -- 2, the default, see
// boost/thread/detail/config.hpp -- boost::thread is *not* a std::thread clone:
//
//   operation                        boost::thread (v2)      std::thread
//   -------------------------------  ----------------------  ----------------
//   join() on a never-started thread no-op, no throw         throws
//   move-assign over a joinable      detaches the old one    std::terminate()
//   destroy a joinable thread        detaches it             std::terminate()
//
// All three measured on this toolchain, and all three are on toffy's hot paths:
// FilterThread::start() assigned unconditionally, ~FilterThread joined
// unconditionally, and Controller::forward() assigned after loopFilters() had
// put the state back to IDLE by itself. So the migration is the two-line part of
// this change and the guards are the substance of it. The tests below are the
// evidence for each guard: some fail on the parent commit, and the ones that
// pass there are marked as pins -- they pass under boost because boost let the
// bad call through, and they fail on an unguarded std::thread.
//
// What the parent commit does to each test, measured by building this file
// against it in a worktree (see cleanup/order.md, "every fix arrives with a test
// that fails on its parent commit"):
//
//   LaneWork.RunsFramesRoundTrip             FAIL "stalled after 18 of 2000
//                                            frames" -- 0 of 2000 under TSan
//   LaneWakeup.DequeueIsReleasedByStop       FAIL: stop() notified inCond only,
//                                            the consumer never came back
//   LaneWakeup.StoppedLane...NotAHang        FAIL: filter() never returned
//   LaneShutdown.StartTwiceKeepsOneWorker    FAIL: two live worker threads, and
//                                            the process then does not exit --
//                                            the detached one cannot be stopped
//   ParallelFilterMux.WithoutABarrier...     SIGSEGV (uninitialised `mux`, heap
//                                            dirtied first)
//   BankReady.TwoPassesAreOneArrival         FAIL: the second try_wait() is
//                                            true; the semaphore counted
//   LaneWork.ReleasesTheFramesItCreated      passes, and LSan reports
//                                            "Direct leak of 240 byte(s) in 3
//                                            object(s)" for init(3)
//   LaneShutdown.NeverStartedLaneIsNotAnError   passes -- boost ignored the join
//   LaneShutdown.ParallelFilterTearsDown...    passes, same reason
//   BankReady.WaitIsReleasedByAPass          passes; the semaphore's wait did
//                                            work, it was its semantics that did
//                                            not
//
// The four that pass there are not evidence that the parent commit was fine --
// the two LaneShutdown ones pass because boost let the join through, and both
// abort with "terminate called after throwing an instance of 'std::system_error'
// what(): Invalid argument" when the joinable() guard is deleted from the fixed
// tree and it is rebuilt (verified). BankReady.WaitIsReleasedByAPass passes on
// both sides because the semaphore's wait() worked; it was its semantics that
// did not, and that is what its neighbour measures.
//
// Two notes on style. First: every call that could block goes through within(),
// because the failure mode being tested for is a hang, and an unbounded
// assertion hangs CI instead of failing it. Second: the lane's frames are raw
// pointers on purpose -- the lane owns them and hands them out to borrow, which
// is the contract ~FilterThread implements and LaneReleasesTheFramesItCreated
// checks with the leak sanitizer.

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <dirent.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>

#include <toffy/filter.hpp>
#include <toffy/filterbank.hpp>
#include <toffy/filterfactory.hpp>
#include <toffy/filterThread.hpp>
#include <toffy/frame.hpp>
#include <toffy/parallelFilter.hpp>

using toffy::FilterFactory;
using namespace std::chrono_literals;

namespace
{
/** A filter that counts how many frames went through it. */
class LaneFilter : public toffy::Filter
{
   public:
    LaneFilter() : toffy::Filter("threadlane") { bank(nullptr); }

    bool filter(const toffy::Frame& /*in*/, toffy::Frame& /*out*/) override
    {
        runs_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    int runs() const { return runs_.load(std::memory_order_relaxed); }

   private:
    std::atomic<int> runs_{0};
};

/** Registers the lane filter type; idempotent, like the dummy helper's. */
void registerLaneFilter()
{
    FilterFactory::registerCreator(
        "threadlane", []() -> toffy::Filter* { return new LaneFilter(); });
}

/** Creates a LaneFilter through the factory, keeping shared ownership. */
std::shared_ptr<LaneFilter> makeLaneFilter()
{
    registerLaneFilter();
    return std::static_pointer_cast<LaneFilter>(
        FilterFactory::getInstance()->createFilter("threadlane"));
}

/**
 * Live threads of this process, from /proc.
 *
 * The point is to see a detached thread: it has no handle anywhere in toffy, so
 * a thread count is the only thing that can report it. POSIX-only, like the rest
 * of the tree since P2-2, and CI is Linux.
 */
std::size_t liveThreads()
{
    std::size_t n = 0;
    DIR* tasks = opendir("/proc/self/task");
    if (!tasks) return 0;   // not Linux: the caller's expectations get weaker
    while (struct dirent* e = readdir(tasks))
    {
        if (e->d_name[0] != '.') ++n;
    }
    closedir(tasks);
    return n;
}

/**
 * Run fn() on a worker and report whether it came back within `limit`.
 *
 * A hang is the defect under test, so it must not be able to take the test
 * binary with it. On timeout the worker is detached -- it is blocked in a call
 * that should have returned, and the assertion the caller raises next is the
 * report. Objects the lambda captures must therefore outlive the test, which is
 * why the lanes here are heap-allocated and released on this path.
 */
template <class F>
bool within(std::chrono::milliseconds limit, F&& fn)
{
    std::atomic<bool> done{false};
    std::thread t([&done, &fn] {
        fn();
        done.store(true);
    });
    const int ticks = static_cast<int>(limit / 10ms) + 1;
    for (int i = 0; i < ticks && !done.load(); ++i)
        std::this_thread::sleep_for(10ms);
    if (!done.load())
    {
        t.detach();
        return false;
    }
    t.join();
    return true;
}

/**
 * ParallelFilter::handleConfigItem() is protected, and the config path that
 * should reach it does not: a <parallelFilter> node inside <toffy> goes through
 * FilterBank::instantiateFilter(), which hands the node to
 * FilterBank::loadConfig() -- and that wants a <toffy> root of its own, logs
 * "Missing root node <toffy>", returns -1, and is not checked there. So a
 * configured parallelFilter currently gets no lanes at all. Filed as A30 in
 * cleanup/findings/correctness.md; the tests below call handleConfigItem()
 * directly so the threading is testable without waiting for that.
 */
class ConfigurableParallelFilter : public toffy::ParallelFilter
{
   public:
    using toffy::ParallelFilter::handleConfigItem;
};

/**
 * A ptree holding one <thread> node running one lane filter.
 *
 * Registers the type as it goes: the config names "threadlane", and the factory
 * creates filters by looking that name up, so a test that builds this config
 * without ever calling makeLaneFilter() would otherwise get an empty lane back.
 */
boost::property_tree::ptree oneLaneConfig()
{
    registerLaneFilter();
    boost::property_tree::ptree pt;
    boost::property_tree::ptree& thread =
        pt.add_child("thread", boost::property_tree::ptree());
    thread.add_child("threadlane", boost::property_tree::ptree())
        .put("name", "lane0");
    return pt;
}
}  // namespace

// ---------------------------------------------------------------------------
// The guards the std::thread migration needs
// ---------------------------------------------------------------------------

// ~FilterThread used to call theThread.join() unconditionally. boost::thread
// ignored that; std::thread throws std::system_error, out of a destructor, which
// is std::terminate(). ~ParallelFilter calls stop() on every lane and destroys
// every lane, including the ones that were configured but never started, so this
// is the normal teardown of any ParallelFilter, not an edge case.
//
// PASSES on the parent commit -- boost let the call through. It fails on an
// unguarded std::thread join, which is what makes it the fence for the guard.
TEST(LaneShutdown, NeverStartedLaneIsNotAnError)
{
    auto f = makeLaneFilter();
    auto lane = std::make_unique<toffy::FilterThread>(f);
    EXPECT_TRUE(within(5s, [&lane] { lane.reset(); }))
        << "destroying a lane that never started did not come back";
}

// The same teardown through the public API: a ParallelFilter whose lanes were
// created by configuration and never run.
TEST(LaneShutdown, ParallelFilterTearsDownWithoutStarting)
{
    auto cfg = oneLaneConfig();
    auto pf = std::make_unique<ConfigurableParallelFilter>();
    ASSERT_GT(pf->handleConfigItem("", cfg.begin()), 0);
    ASSERT_EQ(pf->size(), 1u) << "the lane's filter is pipelined into the bank";
    EXPECT_TRUE(within(5s, [&pf] { pf.reset(); }));
}

// start() used to assign to theThread whatever its state was. With a worker
// already running that detached it: the lane then had two loops pulling from one
// input queue, and only the newest one was reachable by stop() or the
// destructor. std::thread would call std::terminate() on the same assignment, so
// start() refuses instead -- and the thread count is how you can tell.
TEST(LaneShutdown, StartTwiceKeepsOneWorker)
{
    auto f = makeLaneFilter();
    toffy::FilterThread lane(f);

    const std::size_t before = liveThreads();
    lane.start();
    // Wait for the worker to appear, then ask for a second one.
    for (int i = 0; i < 200 && liveThreads() <= before; ++i)
        std::this_thread::sleep_for(10ms);
    ASSERT_GT(liveThreads(), before) << "the first worker never came up";

    lane.start();
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(liveThreads(), before + 1)
        << "a second start() added a worker thread; the first one is detached "
           "and nobody can stop it";
    lane.stop();
}

// ---------------------------------------------------------------------------
// Waking up
// ---------------------------------------------------------------------------

// stop() used to notify inCond only. A consumer parked in dequeue() waits on
// outCond, and once the worker has been told to stop the frame it is waiting for
// can never arrive: the thread is blocked with no way out, and ~FilterThread's
// join() is behind it.
TEST(LaneWakeup, DequeueIsReleasedByStop)
{
    auto f = makeLaneFilter();
    auto lane = std::make_unique<toffy::FilterThread>(f);
    lane->start();   // no frames queued: nothing will ever come out of this one

    std::atomic<toffy::Frame*> got{reinterpret_cast<toffy::Frame*>(1)};
    std::atomic<bool> returned{false};
    std::thread consumer(
        [&] { got.store(lane->dequeue()); returned.store(true); });

    std::this_thread::sleep_for(50ms);
    ASSERT_FALSE(returned.load())
        << "dequeue() returned without any frame being processed";

    lane->stop();
    if (!within(5s, [&returned] {
            while (!returned.load()) std::this_thread::sleep_for(5ms);
        }))
    {
        consumer.detach();
        lane.release();   // cannot be destroyed: its consumer is still inside it
        FAIL() << "stop() did not release the consumer waiting in dequeue()";
    }
    consumer.join();
    EXPECT_EQ(got.load(), nullptr)
        << "a stopped lane must report that there is no frame, not invent one";
}

// The same deadlock through the ParallelFilter API: a stopped lane asked for a
// frame. filter() must report that the pass produced nothing.
TEST(LaneWakeup, StoppedLaneReportsFailureNotAHang)
{
    auto cfg = oneLaneConfig();
    ConfigurableParallelFilter pf;
    ASSERT_GT(pf.handleConfigItem("", cfg.begin()), 0);
    // Deliberately no init(): the lane has no frames, so its worker can only
    // wait, and once it has been stopped nothing can ever produce a frame. With
    // init() the worker might or might not have processed one before it observed
    // the stop -- a legitimate race in the lane, and not what this test is about.
    pf.start();
    pf.stop();

    toffy::Frame in, out;
    bool ran = false;
    bool completed = within(5s, [&] { ran = pf.filter(in, out); });
    EXPECT_TRUE(completed) << "filter() on a stopped lane blocked forever";
    if (completed)
    {
        EXPECT_FALSE(ran) << "a stopped lane reported a successful pass";
    }
}

// ---------------------------------------------------------------------------
// The lane still works
// ---------------------------------------------------------------------------

// A frame goes in, the filter runs, the result comes out, the frame goes back.
// 2000 round trips, because the pre-fix wait used a mutex built on the spot
// inside the waiting function -- a lock no other thread ever held -- so the
// emptiness test and the wait were not atomic with respect to enqueue() and a
// frame pushed in that window was notified to nobody.
TEST(LaneWork, RunsFramesRoundTrip)
{
    auto f = makeLaneFilter();
    toffy::FilterThread lane(f);
    lane.init(2);
    lane.start();

    const int total = 2000;
    std::atomic<int> roundTrips{0};
    bool finished = within(30s, [&] {
        while (roundTrips.load() < total)
        {
            toffy::Frame* fr = lane.dequeue();
            if (!fr) return;
            roundTrips.fetch_add(1);
            lane.enqueue(fr);
        }
    });
    lane.stop();

    EXPECT_TRUE(finished) << "the round trip stalled after " << roundTrips.load()
                          << " of " << total << " frames";
    EXPECT_EQ(roundTrips.load(), total);
    EXPECT_GE(f->runs(), total);
}

// The lane owns the Frames it creates in init(); ~FilterThread used to call
// clear() on both queues, which drops the elements and leaks the Frames. Nothing
// here can observe that -- it is the leak sanitizer's subject, and this test runs
// in the `sanitizers` CI job, where the parent commit reports the Frames as
// directly lost.
TEST(LaneWork, ReleasesTheFramesItCreated)
{
    auto f = makeLaneFilter();
    {
        toffy::FilterThread lane(f);
        lane.init(3);
        lane.start();
        lane.stop();
    }
    SUCCEED();
}

// ---------------------------------------------------------------------------
// FilterBank's frame-arrival signal
// ---------------------------------------------------------------------------

// FilterBank::wait()/try_wait() used to be a
// boost::interprocess::interprocess_semaphore that counted every pass while
// nothing in the tree ever called wait(). The count therefore only climbed, and
// a consumer that turned up after two passes got two immediate returns -- "two
// frames have gone by at some point", not "a frame has arrived since you last
// looked". One flag now: set by a pass, consumed by whoever reads it.
TEST(BankReady, TwoPassesAreOneArrival)
{
    toffy::FilterBank bank;
    toffy::Frame in, out;

    EXPECT_FALSE(bank.try_wait()) << "a fresh bank has not filtered anything";
    ASSERT_TRUE(bank.filter(in, out));
    ASSERT_TRUE(bank.filter(in, out));
    ASSERT_TRUE(bank.try_wait()) << "the first read sees that a pass happened";
    EXPECT_FALSE(bank.try_wait())
        << "the signal is consumed. The semaphore counted, so on the parent "
           "commit this second read is true as well: two passes, two arrivals.";
}

// The other half of the same primitive: wait() blocks until a pass happens. This
// is the call the interprocess semaphore was there for, and the one whose wakeup
// must not be lost -- the flag is set under the mutex the waiter tests.
TEST(BankReady, WaitIsReleasedByAPass)
{
    toffy::FilterBank bank;
    toffy::Frame in, out;
    std::atomic<bool> returned{false};
    std::thread consumer([&bank, &returned] {
        bank.wait();
        returned.store(true);
    });

    std::this_thread::sleep_for(50ms);
    ASSERT_FALSE(returned.load()) << "wait() returned before anything was filtered";

    bank.filter(in, out);
    bool woken = within(5s, [&returned] {
        while (!returned.load()) std::this_thread::sleep_for(5ms);
    });
    if (!woken)
    {
        consumer.detach();   // blocked in a wait() that nothing can satisfy
        FAIL() << "wait() was not released by a filter() pass";
    }
    consumer.join();
}

// ---------------------------------------------------------------------------
// ParallelFilter
// ---------------------------------------------------------------------------

// ParallelFilter's constructor did not initialise `mux`, so a bank configured
// without a <barrier> read an indeterminate pointer in filter()'s `if (mux)` and
// could call through it. Same defect class as P0-5 (uninitialised members),
// found while reading the class for P2-7 and fixed with an `= nullptr`.
//
// The heap is dirtied first: whether the parent commit crashes here depends on
// what malloc left behind, which is why this is not usually reported as a bug.
// With the member initialised the test cannot crash, dirty heap or not.
TEST(ParallelFilterMux, WithoutABarrierThereIsNothingToCall)
{
    // Fill a block the size of a ParallelFilter with a plausible non-null
    // pointer and free it, so the allocation below is likely to land on it.
    const std::size_t n = sizeof(toffy::ParallelFilter);
    void* block = ::operator new(n);
    for (std::size_t i = 0; i + sizeof(std::uintptr_t) <= n;
         i += sizeof(std::uintptr_t))
    {
        *reinterpret_cast<std::uintptr_t*>(
            static_cast<char*>(block) + i) = 0x1000;
    }
    ::operator delete(block);

    auto cfg = oneLaneConfig();
    ConfigurableParallelFilter pf;
    ASSERT_GT(pf.handleConfigItem("", cfg.begin()), 0);
    pf.init();
    pf.start();

    toffy::Frame in, out;
    EXPECT_TRUE(within(10s, [&] { pf.filter(in, out); }))
        << "filter() did not come back";
    pf.stop();
}
