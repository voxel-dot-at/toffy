#pragma once
//#include <string>
//#include <list>

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
	 * @brief Constructor: the filter passed is from then owned by FT and
	 * will be deleted in the destructor.
	 * @param filter
	 */
    FilterThread(Filter* filter) : f(filter) {}

    /**
	 * @brief ~FilterThread
	 */
    virtual ~FilterThread();

    /**
	 * @brief FilterThread
	 * @param ft
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
    Filter* f; ///< @todo document

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
