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

#include "toffy/base/cond.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif

static std::string theName("cond");
static int counter=0;

Cond::Cond(): FilterBank(theName, counter++)
{
}


bool Cond::filter(const Frame &in, Frame& out) 
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    if (enabled) {
	return FilterBank::filter(in, out);
    }

    return true;
}

boost::property_tree::ptree Cond::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    return pt;
}
