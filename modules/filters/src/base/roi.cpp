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
#include <opencv2/highgui.hpp>

#include <boost/log/trivial.hpp>

#include "toffy/base/roi.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;

std::size_t Roi::_filter_counter = 1;
const std::string Roi::id_name = "roi";

toffy::filters::Roi::Roi(): Filter(Roi::id_name,_filter_counter),
    _in_img("img"), _out_img(_in_img), _x(0), _y(0),
    _width(0), _height(0), _inValue(0.0), _outValue(0.0),
    _in(false), _filter(false), _below(false)
{
    _filter_counter++;
}

void toffy::filters::Roi::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _x = pt.get<double>("options.x",_x);
    _y = pt.get<double>("options.y",_y);
    _width = pt.get<double>("options.width",_width);
    _height = pt.get<double>("options.height",_height);
    _inValue = pt.get<double>("options.inValue",_inValue);
    _outValue = pt.get<double>("options.outValue",_outValue);
    _in = pt.get<bool>("options.in",_in);
    _filter = pt.get<bool>("options.filter",_filter);
   // cout << "options.below: " << pt.get<bool>("options.below") << endl;
    _below = pt.get<bool>("options.below",_below);
    //cout << "_below: " << _below << endl;
    _roi = cv::Rect(_x,_y,_width,_height),
    //cout << "set roi: " << _roi << endl;

    _in_img = pt.get<string>("inputs.img",_in_img);
    // Read value or set input
    _out_img = pt.get<string>("outputs.img",_in_img);

}

bool toffy::filters::Roi::filter(const Frame &in, Frame& out) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

	boost::shared_ptr<cv::Mat> img;
	try {
		img = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_in_img));
	} catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Could not cast input " << _in_img <<
			", filter  " << id() <<" not applied.";
		return false;
	}

	boost::shared_ptr<cv::Mat> img_out;
	if (_in_img != _out_img) {
	    try {
		    img_out = in.getMatPtr(_out_img);
	    } catch(const boost::bad_any_cast &) {
		    BOOST_LOG_TRIVIAL(info) <<
			    "Could not cast input " << _out_img;
		    img_out.reset(new cv::Mat(img->size(), img->type()));
		    out.addData(_out_img,img_out);
	    }
	    *img_out=img->clone();

	} else
	    img_out = img;

	if(_roi.area()) {
	    cv::Rect rect_mat(0, 0, img_out->cols, img_out->rows);
	    if ((_roi & rect_mat) == _roi) {
		if (_in) {
		    Mat dst_roi = (*img_out)(_roi);
		    if(_filter) {
			Mat fmask;
			if (_below) {
			    fmask = dst_roi < _inValue;
			    //imshow("below", fmask);
			} else {
			    fmask = dst_roi > _inValue;
			    //imshow("above", fmask);
			}
			dst_roi.setTo(_outValue,fmask);
		    } else
			dst_roi = _outValue;
		    //img_out->copyTo(*img_out,dst_roi);
		} else {
		    //cv::Mat outliers = cv::Mat(img_out->size(), img_out->type(),_outValue);
		    //Mat imgRoi = (*img_out)(_roi);
		    //Mat dst_roi = outliers(_roi);
		    if(_filter) {
			Mat fmask;
			if (_below) {
			    fmask = *img_out > _inValue;
			    //imshow("below", fmask);
			} else {
			    fmask = *img_out < _inValue;
			    //imshow("above", fmask);
			}
			Mat copyImg = img_out->clone();
			copyImg.setTo(_outValue,fmask);
			(*img_out)(_roi).copyTo((copyImg)(_roi));
			*img_out = copyImg.clone();
		    } else {
			cv::Mat outliers = cv::Mat(img_out->size(), img_out->type(),_outValue);
			(*img_out)(_roi).copyTo(outliers(_roi));
			*img_out = outliers.clone();
		    }
		}
	    } else
		BOOST_LOG_TRIVIAL(info) <<
			"Roi does not match image: img: " << rect_mat << " roi: " <<
			_roi << ". Skipping... .Filter  " <<
			id() << " not applied.";
	} else
	    BOOST_LOG_TRIVIAL(info) <<
		    "Roi is null. Skipping... .Filter  " <<
		    id() << " not applied.";

	/*if (_in_img != _out_img) {
	    boost::shared_ptr<cv::Mat> img_out;
	    try {
		    img_out = in.getMatPtr(_out_img);
	    } catch(const boost::bad_any_cast &) {
		    BOOST_LOG_TRIVIAL(info) <<
			    "Could not cast input " << _out_img;
		    img_out.reset(new cv::Mat(img->size(), img->type()));
	    }
	    *img_out=img->clone();

	}*/

	return true;
}

boost::property_tree::ptree toffy::filters::Roi::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.x", _x);
    pt.put("options.y", _y);
    pt.put("options.width", _width);
    pt.put("options.height", _height);
    pt.put("options.outValue", _outValue);
    pt.put("options.inValue",_inValue);
    pt.put("options.in", _in);
    pt.put("options.filter", _filter);
    pt.put("options.below", _below);

    pt.put("inputs.img", _in_img);

    pt.put("outputs.img", _out_img);


    return pt;
}
