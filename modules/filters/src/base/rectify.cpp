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

#include "toffy/base/rectify.hpp"

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/calib3d.hpp>
#warning eh da
//#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "toffy/common/filenodehelper.hpp"


using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;
namespace fs = boost::filesystem;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif

std::size_t Rectify::_filter_counter = 1;

const std::string Rectify::id_name = "rectify";

Rectify::Rectify(): Filter(Rectify::id_name,_filter_counter),
		    _in_img("img"), _out_img("_in_img"), initialized(false)
{
    _filter_counter++;

}

void Rectify::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    boost::optional<const boost::property_tree::ptree& > ocvo = pt.get_child_optional( "options.cameraMatrix" );
    //cout << "options.cameraMatrix: " << pt.get<string>("options.cameraMatrix") << endl;
    //cout << "options.distCoeffs: " << pt.get<string>("options.distCoeffs") << endl;
    //cout << "options.fovx: " << pt.get<string>("options.fovx") << endl;
    //cout << "options.fovy: " << pt.get<string>("options.fovy") << endl;
    //cout << "options.dis: " << pt.get<string>("options.dis") << endl;

    if (ocvo.is_initialized()) {
	if( toffy::commons::checkOCVNone(*ocvo) ) {
            boost::property_tree::ptree os;
            os.put_child("cameraMatrix",*ocvo);
            cv::FileStorage fs = commons::loadOCVnode(os);
            cout << "OpenCV version : " << CV_VERSION << endl;
            cout << "Major version : " << CV_MAJOR_VERSION << endl;
            cout << "Minor version : " << CV_MINOR_VERSION << endl;
            cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;

            fs["cameraMatrix"] >> _cameraMatrix;
            cout << "_cameraMatrix : " << _cameraMatrix << endl;
            fs.release();
	} else
            BOOST_LOG_TRIVIAL(debug) << "Node cameraMatrix is not opencv.";
    } else
         BOOST_LOG_TRIVIAL(debug) << "Node options.cameraMatrix not found.";

    ocvo = pt.get_child_optional( "options.distCoeffs" );
    if (ocvo.is_initialized()) {
	if( toffy::commons::checkOCVNone(*ocvo) ) {
            boost::property_tree::ptree os;
            os.put_child("distCoeffs",*ocvo);
            cv::FileStorage fs = commons::loadOCVnode(os);
            fs.getFirstTopLevelNode() >> _distCoeffs;
            fs.release();
	} else
            BOOST_LOG_TRIVIAL(debug) << "Node distCoeffs is not opencv.";
    } else
         BOOST_LOG_TRIVIAL(debug) << "Node options.distCoeffs not found.";

    _in_img = pt.get<string>("inputs.img",_in_img);
    _in_cameraMatrix = pt.get<string>("inputs.cameraMatrix",_in_cameraMatrix);
    _in_distCoeffs = pt.get<string>("inputs.distCoeffs",_in_distCoeffs);

    _out_img = pt.get<string>("outputs.img",_out_img);
}

boost::property_tree::ptree Rectify::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();
    pt.put("options.cameraMatrix", _in_cameraMatrix);
    pt.put("options.distCoeffs", _in_distCoeffs);

    pt.put("inputs.img", _in_img);
    pt.put("inputs.cameraMatrix", _cameraMatrix);
    pt.put("inputs.distCoeffs", _distCoeffs);

    pt.put("outputs.img", _out_img);

    return pt;
}

bool Rectify::filter(const Frame &in, Frame& out) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

	boost::shared_ptr<cv::Mat> img, n2;
	try {
		img = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_in_img));

		if ( out.hasKey(_out_img)) {
		    n2 = boost::any_cast<boost::shared_ptr<cv::Mat> >(out.getData(_out_img));
		} else {
		    n2.reset(new cv::Mat());
		}
	} catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Could not cast input " << _in_img <<
			", filter  " << id() <<" not applied.";
		return false;
	}

	if ( !map1.rows || ! initialized) {
	  BOOST_LOG_TRIVIAL(debug) << "rectify: Initializing remap data";
	    if (!_in_cameraMatrix.empty()) {
		try {
		    _cameraMatrix= boost::any_cast<cv::Mat>(in.getData(_in_cameraMatrix));
		} catch(const boost::bad_any_cast &) {
		    BOOST_LOG_TRIVIAL(warning) <<
			"Could not read input " << _in_cameraMatrix;
		}
	    }
	    if (!_in_distCoeffs.empty()) {
		try {
		    _distCoeffs= boost::any_cast<cv::Mat>(in.getData(_in_distCoeffs));
		} catch(const boost::bad_any_cast &) {
		    BOOST_LOG_TRIVIAL(warning) <<
			"Could not read input " << _in_distCoeffs;
		}
	    }

	    if (_cameraMatrix.empty() || _distCoeffs.empty()) {
		BOOST_LOG_TRIVIAL(warning) <<
		    "No cameraMatrix or distCoeffs data, filter " << id() <<" not applied.";
		return false;
	    }

	    initUndistortRectifyMap(_cameraMatrix, _distCoeffs, Mat(), _cameraMatrix, img->size(), CV_32FC1,  map1,  map2);
	    initialized = true;
	}
	//TODO we need new data at the output
	//TODO handle aux vars creation
	Mat& new_img = *img;
	remap(new_img, *n2, map1, map2, INTER_NEAREST);
	
	out.addData(_out_img, n2); 

	return true;
}
