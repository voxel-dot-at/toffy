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
#include <boost/log/trivial.hpp>

#include "toffy/filterThread.hpp"

using namespace toffy;

FilterThread::~FilterThread()
{
    // stop thread, kill all.
    theThread.join();
    inQ.clear();
    outQ.clear(); //TODO: check frames..

    delete f;
}


// init the FT with a number of pre-allocated frames
void FilterThread::init(int numFrames)
{
    for (int i=0;i<numFrames;i++) {
	Frame* f = new Frame();
	inQ.push_back(f);
    }
}


void FilterThread::start()
{
    keepRunning = true;
    //todo: launch thread
    theThread = boost::thread( boost::bind(&FilterThread::loop, this) );

}

void FilterThread::stop()
{
    keepRunning = false;
    inCond.notify_one();
}

Frame* FilterThread::dequeue()
{
    Frame* fr;

    if ( outQ.empty() ) {
	boost::mutex mtx;
	boost::unique_lock<boost::mutex> lock(mtx);

	BOOST_LOG_TRIVIAL(trace) << "FT deq wait";
	outCond.wait(lock);
    }
    outMtx.lock();
    fr = outQ.front();
    outQ.pop_front();
    outMtx.unlock();
    return fr;
}

void FilterThread::enqueue(Frame* fr)
{
    inMtx.lock();
    inQ.push_back(fr);
    inMtx.unlock();

    inCond.notify_one();
}

void FilterThread::loop()
{
    boost::mutex mtx;
    boost::unique_lock<boost::mutex> lock(mtx);
    Frame* in;

    BOOST_LOG_TRIVIAL(info) << "FT thread started " << boost::this_thread::get_id();
    while (keepRunning) {
	while (inQ.empty()) {
	    // trace, not debug: this is reached on every wait in the worker loop.
	    // It used to be `cout << ... << endl`, i.e. a flushing write to stdout
	    // per iteration of a real-time frame loop.
	    BOOST_LOG_TRIVIAL(trace) << "FT wait for data";
	    inCond.wait(lock);
	    if (!keepRunning) {
		BOOST_LOG_TRIVIAL(debug) << "FT loop exit";
		return;
	    }
	}
	BOOST_LOG_TRIVIAL(trace) << "FT get data";
	// get one frame
	inMtx.lock();
	in = inQ.front();
	inQ.pop_front();
	inMtx.unlock();

	BOOST_LOG_TRIVIAL(trace) << "FT run filter on " << in;
	// run the filter
	f->filter(*in, *in);

	BOOST_LOG_TRIVIAL(trace) << "FT push result";
	// post the result
	outMtx.lock();
	outQ.push_back(in);
	outMtx.unlock();
	outCond.notify_all();
    }
    BOOST_LOG_TRIVIAL(info) << "FT thread loop exit " << boost::this_thread::get_id();
}
