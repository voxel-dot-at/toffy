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

#include <opencv2/imgproc.hpp>

#include <boost/log/trivial.hpp>

#include "toffy/base/range.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;

std::size_t toffy::filters::Range::_filter_counter = 1;

toffy::filters::Range::Range(): Filter("range",_filter_counter),
    _in_img("img"), _out_img(_in_img), _min(0), _max(0) {
    _filter_counter++;
}

/*
int toffy::filters::Range::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const boost::property_tree::ptree& pt)";
    const boost::property_tree::ptree& range = pt.get_child(this->type());
    updateConfig(range);

    return true;
}
*/

void toffy::filters::Range::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _min = pt.get<float>("options.min",_min);
    _max = pt.get<float>("options.max",_max);

    _in_img = pt.get<string>("inputs.img",_in_img);
    _out_img = pt.get<string>("outputs.img",_out_img);

}

bool toffy::filters::Range::filter(const Frame &in, Frame& out) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

	matPtr img;
	try {
		img = boost::any_cast<matPtr>(in.getData(_in_img));
	} catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Could not cast input " << _in_img <<
			", filter  " << id() <<" not applied.";
		return false;
	}

	// Mat minMask = *img < _min;
	// Mat maxMask = *img > _max;
	// Mat mask = minMask | maxMask;
	// mask = ~mask;

	Mat mask = (*img >= _min) & (*img <= _max) ;

	if (!img->data) {
	    BOOST_LOG_TRIVIAL(warning) <<
		    "Range filtered out everything from input: " << _in_img <<
		    ", filter  " << id() <<" failed.";
	    return false;
	}


	matPtr img_out;
	try {
		img_out = boost::any_cast<matPtr>(in.getData(_out_img));
	} catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(info) << 
			"Range::filter() Could not cast output " << _out_img << " - initializing it.";
		img_out.reset(new Mat());
		img->copyTo(*img_out, mask);
		out.addData(_out_img,img_out);
		return true;
	}
	//img_out.reset(new Mat());
    *img_out = 0;
	img->copyTo(*img_out, mask);

	return true;
}

boost::property_tree::ptree toffy::filters::Range::getConfig() const
{
    boost::property_tree::ptree pt, opt;

    pt = Filter::getConfig();

    opt.put("min", _min);
    opt.put("max", _max);
    pt.add_child("options", opt);
    opt.clear();

    opt.put("img", _in_img);
    pt.add_child("inputs", opt);
    opt.clear();

    opt.put("img", _out_img);
    pt.add_child("outputs", opt);

    return pt;
}
