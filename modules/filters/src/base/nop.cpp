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

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <boost/log/trivial.hpp>

#include "toffy/base/nop.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;

#ifdef CM_DEBUG
static const bool dbg=true;
#else
static const bool dbg=false;
#endif


Nop::Nop(): Filter("nop")
{
}

/*
int Nop::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const boost::property_tree::ptree& pt)";
    const boost::property_tree::ptree& nop = pt.get_child(this->type());

    loadGlobals(nop);

    updateConfig(nop);

    return true;
}

void Nop::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    setCommon(pt);

}
*/

bool Nop::filter(const Frame &in, Frame& out) 
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    out = in;

    return true;
}

boost::property_tree::ptree Nop::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    return pt;
}
