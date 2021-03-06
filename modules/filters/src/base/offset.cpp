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
#include <string>
#include <iostream>
#include <fstream>
#include <limits.h>

#include <boost/log/trivial.hpp>
//#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/foreach.hpp>

#include "toffy/base/offset.hpp"



using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;
//namespace fs = boost::filesystem;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif

std::size_t OffSet::_filter_counter = 1;
const std::string OffSet::id_name = "offset";

OffSet::OffSet(): Filter(OffSet::id_name,_filter_counter),
		_sumValue(0.0),
		_mulValue(1.0),
		_in_img("img")
{
    _filter_counter++;
}

/*
int OffSet::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const boost::property_tree::ptree& pt)";
    const boost::property_tree::ptree& tree = pt.get_child(this->type());

    loadGlobals(tree);

    updateConfig(tree);

    return true;
}
*/

void OffSet::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _in_img = pt.get<string>("inputs.img",_in_img);

    _sumValue = pt.get<float>("options.sumValue",_sumValue);
    _mulValue = pt.get<float>("options.mulValue",_mulValue);

    _out_img = pt.get<string>("outputs.img", _out_img);

    try {
	BOOST_FOREACH(const ptree::value_type &v, pt.get_child("options.roi")) {
	    Rect roi(0,0,0,0);
	    roi.x = v.second.get<int>("x",roi.x);
	    roi.y = v.second.get<int>("y",roi.y);
	    roi.width = v.second.get<int>("width",roi.width);
	    roi.height = v.second.get<int>("height",roi.height);
	    _rois.push_back(roi);
	}
    } catch (const std::exception& ex) {
	BOOST_LOG_TRIVIAL(debug) << "Roi values are wrong. Ex:" << ex.what();
    }
}

/*int OffSet::loadConfig(const FileNode &fn) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

	Filter::loadConfig(fn);

	FileNode offset = fn;

	offset["sumValue"] >> _sumValue;
	offset["mulValue"] >> _mulValue;
	BOOST_LOG_TRIVIAL(debug) << "sum_val: " << _sumValue;
	BOOST_LOG_TRIVIAL(debug) << "mul_val: " << _mulValue;

	//Look for inputs
	FileNode ios = offset["inputs"];
	if (ios.empty()) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Missing inputs for filter " <<	id() <<
			" ... using defauts: " << endl <<
			"img: " << _in_img;
	} else {
		ios["img"] >> _in_img;
		BOOST_LOG_TRIVIAL(debug) << "input img: " << _in_img;
	}


	ios = offset["outputs"];
	if (ios.empty()) {
		_out_img = _in_img;
		BOOST_LOG_TRIVIAL(warning) <<
			"Missing outputs for filter " <<	id() <<
			" ... using same as input: " << endl <<
			"_out_img: " << _out_img;
	} else {
		ios["img"] >> _out_img;
		BOOST_LOG_TRIVIAL(debug) << "output img: " << _out_img;
	}

	FileNode rois = offset["rois"];
	for(FileNodeIterator it = rois.begin(); it != rois.end(); ++it)	{
		Rect roi;
		(*it)["x"] >> roi.x;
		(*it)["y"] >> roi.y;
		(*it)["width"] >> roi.width;
		(*it)["height"] >> roi.height;
		_rois.push_back(roi);
	}
	return 1;
}*/

bool OffSet::filter(const Frame &in, Frame& out) const {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
	out = in;

	if (_rois.size() == 0) {
		BOOST_LOG_TRIVIAL(debug) << "depth found";
		boost::shared_ptr<cv::Mat> d = in.getMatPtr(_in_img);
		*d += _sumValue;
		*d *= _mulValue;
	} else {

		for(size_t i = 0; i < _rois.size(); i++) {
			boost::shared_ptr<cv::Mat> d = in.getMatPtr(_in_img);
			Mat roi((*d)(_rois[i]));
			roi += _sumValue;
			roi *= _mulValue;
		}
	}
	return true;
}

void OffSet::addRoi(cv::Rect newRoi) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
	_rois.push_back(newRoi);
}

boost::property_tree::ptree OffSet::getConfig() const
{
    boost::property_tree::ptree pt, opt;

    pt = Filter::getConfig();

    opt.put("sumValue", _sumValue);
    opt.put("mulValue", _mulValue);
    pt.add_child("options", opt);
    opt.clear();

    opt.put("img", _in_img);
    pt.add_child("inputs", opt);
    opt.clear();

    opt.put("img", _out_img);
    pt.add_child("outputs", opt);

    return pt;
}



