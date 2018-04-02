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
