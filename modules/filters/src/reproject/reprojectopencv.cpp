#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#  include <opencv2/calib3d.hpp>
#else
#  include <opencv2/core/core.hpp>
#  include <opencv2/calib3d/calib3d.hpp>
#endif

#include <boost/log/trivial.hpp>
#include <boost/any.hpp>

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
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

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
	} else
	    BOOST_LOG_TRIVIAL(debug) << "Node cameraMatrix is not opencv.";
    } else
	BOOST_LOG_TRIVIAL(debug) << "Node options.cameraMatrix not found.";
}

boost::property_tree::ptree ReprojectOpenCv::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("inputs.img", _in_img);

    pt.put("outputs.cloud", _out_cloud);

    return pt;
}

bool ReprojectOpenCv::filter(const Frame &in, Frame& out) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    matPtr img;
    try {
	img = in.getMatPtr(_in_img);
    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(error) << "Could not cast input " << _in_img <<
				      ", filter  " << id() <<" not applied.";
	return false;
    }

    matPtr img3d;
    try {
	img3d = in.getMatPtr(_out_cloud);
    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) << "Could not cast input " << _out_cloud;
	img3d.reset(new Mat(img->size(),CV_32FC3));
    }
    if(img3d->size() != img->size()) {
	img3d.reset(new Mat(img->size(),CV_32FC3));
    }
    //matPtr img3d (new Mat(img->size(),CV_32FC3));

    if (!_in_cameraMatrix.empty()) {
	try {
	    _cameraMatrix= boost::any_cast<cv::Mat>(in.getData(_in_cameraMatrix));
	} catch(const boost::bad_any_cast &) {
	    BOOST_LOG_TRIVIAL(warning) <<
					  "Could not read input " << _in_cameraMatrix;
	}
    }

    double fl_x_reciprocal, fl_y_reciprocal;
    Point2d center;
    if(_cameraMatrix.data) {
	//Saving parameter from the camera matrix
	fl_x_reciprocal = 1.0f / _cameraMatrix.at<double>(0,0);
	fl_y_reciprocal = 1.0f / _cameraMatrix.at<double>(1,1);
	//center_x = _cameraMatrix.at<double>(0,2);
	//center_y = _cameraMatrix.at<double>(1,2);

	double noV,
		apertureWidth = (45/1000)*img->size().width,
		apertureHeight = (45/1000)*img->size().height;
	calibrationMatrixValues(_cameraMatrix, img->size(),
				apertureWidth, apertureHeight,
				noV, noV, noV, center, noV);
    } else {
	BOOST_LOG_TRIVIAL(warning) <<"No cameraMatrix data, filter " <<
				     id() <<" not applied.";
	return false;
    }

    float *dptr, depthValue;
    // Calculates the 3D point for each depth value
    for (int y=0; y < img->rows; ++y) {
	for (int x=0; x < img->cols; ++x) {
	    //PointWithRange& point = getPointNoCheck (x, y);
	    dptr = img3d->ptr<float>(y,x);
	    depthValue = img->at<float>(y,x);

	    //Filtering by roi, min and max distance and by amplitudes
	    if (depthValue <= 0.0f || depthValue > 65.0f) {
		dptr[0] = std::numeric_limits<float>::quiet_NaN (); //X
		dptr[1] = std::numeric_limits<float>::quiet_NaN (); //Y
		dptr[2] = std::numeric_limits<float>::quiet_NaN (); //Y
		//cout << " Filtered (" << x << "," << y << ")" << endl;
		continue;
	    }
	    //Set 3D points
	    dptr[0] = (static_cast<float> (x) - center.x) * depthValue * fl_x_reciprocal; //X
	    dptr[1] = (static_cast<float> (y) - center.y) * depthValue * fl_y_reciprocal; //Y
	    dptr[2] = depthValue; //Z
	}
    }
    out.addData(_out_cloud,img3d);

    return true;
}

