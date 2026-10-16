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

#include "toffy/filterbank.hpp"
#include "toffy/filterThread.hpp"
#include "toffy/mux.hpp"

#include "toffy/parallelFilter.hpp"

using namespace std;
using namespace toffy;

std::size_t ParallelFilter::_filter_counter = 1;
const std::string ParallelFilter::id_name = "parallelFilter";

ParallelFilter::~ParallelFilter()
{
    for (size_t i = 0; i < lanes.size(); i++) {
        lanes[i]->stop();
        delete lanes[i];
    }
}

int ParallelFilter::handleConfigItem(
    const std::string& confFile,
    const boost::property_tree::ptree::const_iterator& it)
{
    // The `cout << "PF::handle "` that followed this logged the same two values
    // as the line above, so it is deleted rather than converted.
    BOOST_LOG_TRIVIAL(debug)
        << __FUNCTION__ << ":: " << type() << " " << it->first;

    if (it->first == "parallelFilter") {
        // recurse
        return loadConfig(confFile, it->second.begin(), it->second.end());
    } else if (it->first == "thread") {
        BOOST_LOG_TRIVIAL(debug) << "PF::thread node";
        // FilterPtr, not Filter* (P2-5): this filter ends up owned by two
        // places -- the FilterThread that runs it and this bank's pipeline --
        // and that is exactly what shared ownership is for. With raw pointers
        // it was a double free: ~ParallelFilter deleted the lanes (each lane
        // deleting its filter) and then ~FilterBank cleared _pipe through the
        // factory, which deleted the same object again.
        FilterPtr f;
        if (it->second.size() > 1) {
            // instantiate a filterbank
            //
            // Held as a shared_ptr<FilterBank> rather than a raw FilterBank*
            // and moved into `f` at the end: the loadConfig() below can throw,
            // and a raw local meant the lane's bank leaked out of that
            // exception. It stays typed as a FilterBank rather than as the
            // FilterPtr `f`, because loadConfig()/loadFileConfig() on a bank
            // are the protected virtuals that FilterBank redeclares.
            std::shared_ptr<FilterBank> fb = std::make_shared<FilterBank>();

            // bank(NULL), not bank(this): FilterBank's own constructor sets the
            // self-reference, and getBaseFilterbank() walks it, so leaving it
            // pointed at the ParallelFilter would recurse through the
            // ParallelFilter's own self-reference. Unchanged behaviour.
            fb->bank(NULL);
            fb->loadConfig(it->second);

            f = std::move(fb);
        } else if (it->second.begin()->first == "filterGroup") {
            // instantiate a filterbank
            std::shared_ptr<FilterBank> fb = std::make_shared<FilterBank>();
            const boost::property_tree::ptree pt = it->second.begin()->second;

            BOOST_LOG_TRIVIAL(debug) << "PF::filterGroup file " << pt.data();
            fb->bank(NULL);  // see the note above
            fb->loadFileConfig(pt.data());

            f = std::move(fb);
        } else {
            const boost::property_tree::ptree pt = it->second;
            f = instantiateFilter(pt.begin());
        }

        if (!f) {
            // instantiateFilter() returns empty for an unknown type; the lane
            // would then dereference it in its own thread.
            BOOST_LOG_TRIVIAL(error)
                << name() << "::" << __FUNCTION__
                << " could not create the filter for this thread node";
            return 0;
        }

        FilterThread* ft = new FilterThread(f);
        lanes.push_back(ft);

        add(f);  // book-keeping

    } else if (it->first == "barrier") {
        boost::property_tree::ptree::const_iterator child = it->second.begin();
        BOOST_LOG_TRIVIAL(debug) << "PF::barrier node " << child->first;

        FilterPtr f = instantiateFilter(child);
        if (!f) {
            BOOST_LOG_TRIVIAL(error)
                << name() << "::" << __FUNCTION__
                << " could not create the barrier filter";
            return 0;
        }

        f->loadConfig(it->second);

        // Observer into _pipe: the mux is owned by the pipeline, and `mux` is a
        // raw member on purpose -- making it a FilterPtr would give the bank a
        // second reference to a filter it already has and no way to tell them
        // apart when one is removed.
        this->mux = static_cast<Mux*>(f.get());

        add(f);  // book-keeping

    } else {
        // return FilterBank::handleConfigItem(confFile, it);
    }
    return 1;
}
//@TODO: TEST! works right now for filterbanks with integrated image source (bta), but not if input frame is from extern.
bool ParallelFilter::filter(const Frame& /*in*/, Frame& out)
{
    std::vector<Frame*> res;
    size_t i;
    // sync all threads to get one result
    res.resize(lanes.size());
    for (i = 0; i < lanes.size(); i++) {
        BOOST_LOG_TRIVIAL(trace) << name() << "::filter dequeuing lane " << i;
        res[i] = lanes[i]->dequeue();
    }
    BOOST_LOG_TRIVIAL(trace) << name() << "::filter all lanes dequeued";

    if (mux) {
        mux->filter(res, out);
    } else {
        BOOST_LOG_TRIVIAL(debug)
            << name() << "::" << __FUNCTION__ << " --> No Muxer found! ";
    }

    // release frames again
    for (i = 0; i < lanes.size(); i++) {
        lanes[i]->enqueue(res[i]);
    }

    return true;
}

void ParallelFilter::init()
{
    for (size_t i = 0; i < lanes.size(); i++) {
        lanes[i]->init(2);
    }
}

void ParallelFilter::start()
{
    for (size_t i = 0; i < lanes.size(); i++) {
        lanes[i]->start();
    }
}

void ParallelFilter::stop()
{
    for (size_t i = 0; i < lanes.size(); i++) {
        lanes[i]->stop();
    }
}
