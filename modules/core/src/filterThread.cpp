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
#include "toffy/filterThread.hpp"

#include <system_error>

#include <boost/log/trivial.hpp>

#include "toffy/frame.hpp"

using namespace toffy;
using namespace std;

FilterThread::~FilterThread()
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    // Stop the worker even if the caller never called stop(). Without the
    // signal, the join below would wait forever on a thread parked in
    // inCond.wait(), and ~ParallelFilter would hang the process.
    stop();
    // std::thread::join() throws std::system_error when there is no thread to
    // join; boost::thread at BOOST_THREAD_VERSION 2 ignored the call. Letting
    // that exception leave a destructor calls std::terminate, so ask first.
    // A lane that was constructed and never started is the normal case:
    // ~ParallelFilter calls stop() on every lane, started or not.
    if (theThread.joinable()) theThread.join();
    // The lane owns the frames it created in init(); the queues were the only
    // place they were reachable, and ~FilterThread used to call clear() on both
    // of them, which drops the elements and leaks the Frames.
    for (Frame* fr : inQ) delete fr;
    for (Frame* fr : outQ) delete fr;
    inQ.clear();
    outQ.clear();
}

void FilterThread::init(int numFrames)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    lock_guard<mutex> lock(qMtx);
    for (int i = 0; i < numFrames; i++)
    {
        Frame* f = new Frame();
        inQ.push_back(f);
    }
}

void FilterThread::start()
{
    if (keepRunning)
    {
        // Nothing to start. The old body assigned to theThread regardless, which
        // at BOOST_THREAD_VERSION 2 *detached* the running worker: a second loop
        // with no owner, both of them pulling from the same input queue, and the
        // lane's destructor joining only the newest. std::thread would call
        // std::terminate() on the same assignment.
        BOOST_LOG_TRIVIAL(warning) << __FUNCTION__ << ": lane already running";
        return;
    }
    // A lane that was stop()ed leaves a finished but still joinable thread
    // behind; collecting it here is what makes stop() followed by start() legal.
    if (theThread.joinable()) theThread.join();
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    keepRunning = true;
    try
    {
        theThread = std::thread(&FilterThread::loop, this);
    }
    catch (const std::system_error& e)
    {
        // std::thread reports a failed launch by throwing; the old
        // `if (!theThread.joinable())` check could never fire, because
        // boost::thread either started or threw as well.
        keepRunning = false;
        BOOST_LOG_TRIVIAL(error) << "FT could not start the lane thread: " << e.what();
    }
}

void FilterThread::stop()
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    {
        // The write is under the mutex the waiters hold while they test the
        // predicate; setting an atomic outside the lock is enough to lose the
        // wakeup between "predicate is false" and "wait".
        lock_guard<mutex> lock(qMtx);
        keepRunning = false;
    }
    // Wake the worker if it is parked on an empty input queue...
    inCond.notify_all();
    // ...and release anyone waiting in dequeue(). stop() used to notify inCond
    // only, so a consumer blocked on outCond had no way out: the frame it is
    // waiting for can never arrive once the worker has been told to stop.
    outCond.notify_all();
}

Frame* FilterThread::dequeue()
{
    BOOST_LOG_TRIVIAL(trace) << "FT deq wait";
    unique_lock<mutex> lock(qMtx);
    // Wait on the queue that is actually consumed, holding the mutex that
    // guards it, with the emptiness test as the predicate. The old body tested
    // outQ.empty() with no lock at all and then waited on a condition variable
    // using a mutex it had built on the spot -- a lock no other thread ever
    // held, so the notify could (and did) arrive before the wait.
    outCond.wait(lock, [this] { return !outQ.empty() || !keepRunning; });
    if (outQ.empty())
    {
        // Only a stop() can get us here. Returning null is the caller's cue;
        // the old code fell through to outQ.front() on an empty list, which is
        // undefined behaviour on the first thing it did with the result.
        BOOST_LOG_TRIVIAL(debug) << "FT dequeue: lane stopped, no frame";
        return nullptr;
    }
    BOOST_LOG_TRIVIAL(trace) << "FT deq got one";
    Frame* fr = outQ.front();
    outQ.pop_front();
    return fr;
}

void FilterThread::enqueue(Frame* fr)
{
    if (!fr)
    {
        // A null here means the caller is returning a frame it never got, i.e.
        // it carried on after dequeue() reported a stopped lane. Queuing it would
        // put a null in front of the worker, which dereferences it.
        BOOST_LOG_TRIVIAL(warning) << "FT enqueue: ignoring null frame";
        return;
    }
    BOOST_LOG_TRIVIAL(trace) << "FT enq " << fr;
    {
        lock_guard<mutex> lock(qMtx);
        inQ.push_back(fr);
    }
    inCond.notify_one();
}

void FilterThread::loop()
{
    BOOST_LOG_TRIVIAL(info) << "FT thread started " << this_thread::get_id();
    while (true)
    {
        Frame* in = nullptr;
        {
            unique_lock<mutex> lock(qMtx);
            inCond.wait(lock,
                        [this] { return !inQ.empty() || !keepRunning; });
            if (!keepRunning)
            {
                // stop() wins, as it always did: frames still queued are left
                // for ~FilterThread to free rather than being processed by a
                // lane that has been told to stop.
                BOOST_LOG_TRIVIAL(debug) << "FT loop exit";
                break;
            }
            BOOST_LOG_TRIVIAL(trace) << "FT get data";
            in = inQ.front();
            inQ.pop_front();
        }
        BOOST_LOG_TRIVIAL(trace) << "FT run filter on " << in;
        f->filter(*in, *in);
        {
            lock_guard<mutex> lock(qMtx);
            outQ.push_back(in);
        }
        BOOST_LOG_TRIVIAL(trace) << "FT push result";
        outCond.notify_all();
    }
    BOOST_LOG_TRIVIAL(info) << "FT thread loop exit " << this_thread::get_id();
}
