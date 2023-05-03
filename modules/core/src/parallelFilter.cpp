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
#include <iostream>

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
    BOOST_LOG_TRIVIAL(debug)
        << __FUNCTION__ << ":: " << type() << " " << it->first;

    cout << "PF::handle " << it->first << endl;
    if (it->first == "parallelFilter") {
        // recurse
        return loadConfig(confFile, it->second.begin(), it->second.end());
    } else if (it->first == "thread") {
        cout << "PF::THREAD FOUND!" << endl;
        Filter* f;
        if (it->second.size() > 1) {
            // instantiate a filterbank
            FilterBank* fb = new FilterBank();

            fb->bank(NULL);
            fb->loadConfig(it->second);

            f = fb;
        } else if (it->second.begin()->first == "filterGroup") {
            // instantiate a filterbank
            FilterBank* fb = new FilterBank();
            const boost::property_tree::ptree pt = it->second.begin()->second;

            cout << "DD " << pt.data() << endl;
            fb->bank(NULL);
            fb->loadFileConfig(pt.data());

            f = fb;
        } else {
            const boost::property_tree::ptree pt = it->second;
            f = instantiateFilter(pt.begin());
        }

        FilterThread* ft = new FilterThread(f);
        lanes.push_back(ft);

        add(f);  // book-keeping

    } else if (it->first == "barrier") {
        boost::property_tree::ptree::const_iterator child = it->second.begin();
        cout << "PF::BARRIER FOUND! " << child->first << endl;

        Filter* f = instantiateFilter(child);

        f->loadConfig(it->second);

        this->mux = (Mux*)f;

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
        cout << "ParallelFilter::filter.deq " << i << endl;
        res[i] = lanes[i]->dequeue();
    }
    cout << "ParallelFilter::filter.deqed " << endl;

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
