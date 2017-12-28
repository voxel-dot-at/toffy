#include <iostream>

#include <boost/log/trivial.hpp>

#include "toffy/filterThread.hpp"

using namespace toffy;
using namespace std;

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

    //cout << "FT deq " << outQ.size() << endl;

    if ( outQ.empty() ) {
	boost::mutex mtx;
	boost::unique_lock<boost::mutex> lock(mtx);

	//cout << "FT deq wait " << endl;
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

    cout << "FT thread started " << boost::this_thread::get_id() << endl;
    while (keepRunning) {
	while (inQ.empty()) {
	    cout << "FT wait for data" << endl;
	    inCond.wait(lock);
	    if (!keepRunning) {
		cout << "FT loop exit" << endl;
		return;
	    }
	}
	cout << "FT get data" << endl;
	// get one frame
	inMtx.lock();
	in = inQ.front();
	inQ.pop_front();
	inMtx.unlock();

	cout << "FT run filter on " << in << endl;
	// run the filter
	f->filter(*in, *in);

	cout << "FT push result" << endl;
	// post the result
	outMtx.lock();
	outQ.push_back(in);
	outMtx.unlock();
	outCond.notify_all();
    }
    cout << "FT thread loop exit " << boost::this_thread::get_id() << endl;
}
