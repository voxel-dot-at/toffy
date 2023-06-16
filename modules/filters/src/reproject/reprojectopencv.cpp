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

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <boost/any.hpp>

#include "toffy/filter_helpers.hpp"
#include "toffy/common/filenodehelper.hpp"

#include "toffy/reproject/reprojectopencv.hpp"

using namespace toffy;
using namespace cv;

std::size_t ReprojectOpenCv::_filter_counter = 1;
const std::string ReprojectOpenCv::id_name = "reprojectopencv";

ReprojectOpenCv::ReprojectOpenCv():
    Filter(ReprojectOpenCv::id_name,_filter_counter), _in_img("img"),
    _in_cameraMatrix("camera_matrix"), _out_cloud("cloud")
{
    _filter_counter++;
}

void ReprojectOpenCv::updateConfig(const boost::property_tree::ptree &pt) {
    LOG(debug) << __FUNCTION__ ;

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _in_img = pt.get<std::string>("inputs.img",_in_img);

    _out_cloud = pt.get<std::string>("outputs.cloud",_out_cloud);

    boost::optional<const boost::property_tree::ptree& > ocvo =
	    pt.get_child_optional( "options.cameraMatrix" );
    if (ocvo.is_initialized()) {
	if( toffy::commons::checkOCVNone(*ocvo) ) {
	    boost::property_tree::ptree os;
	    os.put_child("cameraMatrix",*ocvo);
	    cv::FileStorage fs = commons::loadOCVnode(os);
	    fs.getFirstTopLevelNode() >> _cameraMatrix;
	    fs.release();
	} else {
	    LOG(warning) << "Node cameraMatrix is not opencv.";
        }
    } else {
	LOG(warning) << "Node options.cameraMatrix not found.";
    }
}

boost::property_tree::ptree ReprojectOpenCv::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("inputs.img", _in_img);

    pt.put("outputs.cloud", _out_cloud);

    return pt;
}

bool ReprojectOpenCv::filter(const Frame &in, Frame& out) {
    LOG(info) << __FUNCTION__ << " " << __LINE__;
    matPtr img;
    matPtr img3d;

    try {
	img = in.getMatPtr(_in_img);
    } catch(const boost::bad_any_cast &) {
	LOG(error) << "Could not cast input " << _in_img <<
				      ", filter  " << id() <<" not applied.";
	return false;
    }

    try {
      img3d = out.getMatPtr(_out_cloud);
    } catch (const boost::bad_any_cast &) {
      LOG(info) << "Initializing output " << _out_cloud;
      img3d.reset(new Mat(img->size(), CV_32FC3));
    }
    if (img3d->size() != img->size()) {
      img3d.reset(new Mat(img->size(), CV_32FC3));
    }
    LOG(info) << __FUNCTION__ << " " << __LINE__ << " in " << img->cols << "x" << img->rows ;

    if (!_in_cameraMatrix.empty() && in.hasKey(_in_cameraMatrix) ) {
	try {
	    _cameraMatrix= boost::any_cast<cv::Mat>(in.getData(_in_cameraMatrix));
	} catch(const boost::bad_any_cast &) {
          LOG(warning) << "Could not read input " << _in_cameraMatrix;
        }
    }

    double fl_x_reciprocal, fl_y_reciprocal;
    Point2d center;
    if (_cameraMatrix.data) {
      // Saving parameter from the camera matrix
      fl_x_reciprocal = 1.0f / _cameraMatrix.at<double>(0, 0);
      fl_y_reciprocal = 1.0f / _cameraMatrix.at<double>(1, 1);
      // center_x = _cameraMatrix.at<double>(0,2);
      // center_y = _cameraMatrix.at<double>(1,2);
      LOG(info) << __LINE__ << "cam mtx rep " << fl_x_reciprocal
                 << "x" << fl_y_reciprocal << " " << _cameraMatrix;

      double noV, apertureWidth = 4.5 , // sensor w/h in mm
                  apertureHeight = 4.5;
       double fovx, fovy,focLen, aspect;
      calibrationMatrixValues(_cameraMatrix, img->size(), apertureWidth,
                              apertureHeight, fovx, fovy,focLen, center, aspect);
      LOG(info) << "cam mtx result fov " << fovx << "x" <<fovy << " flen " << focLen << " asp " << aspect;
    } else {
      LOG(warning) << "No cameraMatrix data, filter " << id()
                   << " not applied.";
      return false;
    }
    LOG(info) << __FUNCTION__ << " " << __LINE__;

    float *dptr, depthValue;
    int go=0,nogo=0;
    // Calculates the 3D point for each depth value
    for (int y = 0; y < img->rows; ++y) {
      for (int x = 0; x < img->cols; ++x) {
        // PointWithRange& point = getPointNoCheck (x, y);
        dptr = img3d->ptr<float>(y, x);
        depthValue = img->at<float>(y, x);

        // Filtering by roi, min and max distance and by amplitudes
        if (depthValue <= 0.0f || depthValue > 65.0f) {
          dptr[0] = std::numeric_limits<float>::quiet_NaN();  // X
          dptr[1] = std::numeric_limits<float>::quiet_NaN();  // Y
          dptr[2] = std::numeric_limits<float>::quiet_NaN();  // Y
          // cout << " Filtered (" << x << "," << y << ")" << endl;
          nogo++;
        } else {
          // Set 3D points
          dptr[0] = (static_cast<float>(x) - center.x) * depthValue *
                    fl_x_reciprocal;  // X
          dptr[1] = (static_cast<float>(y) - center.y) * depthValue *
                    fl_y_reciprocal;  // Y
          dptr[2] = depthValue;       // Z
          go++;
        }
      }
    }
    out.addData(_out_cloud,img3d);
    LOG(info) << __FUNCTION__ << " " << __LINE__ << " g " << go << " " << nogo;

    return true;
}
