#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
//#include "trkapi.hpp"
#include <toffy/toffy_config.h>
#include <boost/log/trivial.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/calib3d.hpp>
#else
#  include <opencv2/calib3d/calib3d.hpp>
#endif

namespace toffy {
namespace commons {

/** helper functions for camera handling; assumes we have standard optics (90deg hfov).
 */

const int hPix=160;
const int vPix=120;
const float hFov=90.;
const float vFov=65.;
const float pixSinAngle = 0.009817319; // = sin( deg2rad(90/160) );
const float pixCosAngle = 0.999951809; // = cos( deg2rad(90/160) );
const double fl_x_reciprocal = 1.0f / 8.8345892962843834e+01;
const double fl_y_reciprocal = 1.0f / 8.8395902341635306e+01;
const double center_x = 7.9460485484676596e+01;
const double center_y = 5.7816728185872989e+01;

static inline float deg2rad(float a) { return a/180.*M_PI; }

static inline double getPixelSize(double distance )
{
    //float maxFinger = 0.01 / (angl*pdis);  //==pixel width
    return pixSinAngle * distance;
}

static inline void depth2xyz(const cv::Point2f& p, float d, cv::Point3f& xyz)
{
    xyz.x = pixSinAngle*(p.x-hPix/2) *  pixSinAngle*(p.y-vPix/2) * d;
    xyz.y = pixSinAngle*(p.x-hPix/2) *  pixCosAngle*(p.y-vPix/2) * d;
    xyz.z = pixCosAngle*(p.x-hPix/2) * d;

    float rho = deg2rad( (p.x-hPix/2) );
    float tht = deg2rad( (p.y-vPix/2) );

    xyz.x = d * sin (rho) * sin(tht) ;
    xyz.y = d * sin (rho) * cos(tht) ;
    xyz.z = d * cos (rho);
}

static inline void depth2xyz(const cv::Point2f& p, float d, cv::Vec3f& xyz)
{
    xyz[0] = pixSinAngle*(p.x-hPix/2) *  pixSinAngle*(p.y-vPix/2) * d;
    xyz[1] = pixSinAngle*(p.x-hPix/2) *  pixCosAngle*(p.y-vPix/2) * d;
    xyz[2] = pixCosAngle*(p.x-hPix/2) * d;
}

/*static inline void depth2xyz(const cv::Point2f& p, float d, track::vec3f& xyz)
{
    float rho = deg2rad( (p.x-hPix/2) );
    float tht = deg2rad( (p.y-vPix/2) );

    xyz.x = d * sin (rho) * sin(tht) ;
    xyz.y = d * sin (rho) * cos(tht) ;
    xyz.z = d * cos (rho);
}*/

static inline cv::Point3d pointTo3D(cv::Point2d point, float depthValue, cv::Mat cameraMatrix, cv::Size imgSize)
{
	//BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << point;

	double fl_x_reciprocal, fl_y_reciprocal;
	cv::Point2d center;
	if(cameraMatrix.data) {
	    //Saving parameter from the camera matrix
	    fl_x_reciprocal = 1.0f / cameraMatrix.at<double>(0,0);
	    fl_y_reciprocal = 1.0f / cameraMatrix.at<double>(1,1);
	    //center_x = _cameraMatrix.at<double>(0,2);
	    //center_y = _cameraMatrix.at<double>(1,2);

	    double noV,
		    apertureWidth = (45/1000)*imgSize.width,
		    apertureHeight = (45/1000)*imgSize.height;
	    cv::calibrationMatrixValues(cameraMatrix, imgSize,
				    apertureWidth, apertureHeight,
				    noV, noV, noV, center, noV);
	} else {
	    BOOST_LOG_TRIVIAL(warning) <<"No cameraMatrix data.";
	    return cv::Point3d();
	}


	cv::Point3d out3Dp;

	//Saving parameter from the camera matrix
	out3Dp.x = (static_cast<float> (point.x) - center.x) * depthValue * fl_x_reciprocal; //X
	out3Dp.y = (static_cast<float> (point.y) - center.y) * depthValue * fl_y_reciprocal; //Y
	out3Dp.z = depthValue; //Z

	return out3Dp;
}

static inline cv::Point2i pointTo2D(cv::Point3d point, cv::Mat cameraMatrix, cv::Size imgSize)
{
	//BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
	double fl_x_reciprocal, fl_y_reciprocal;
	cv::Point2d center;
	if(cameraMatrix.data) {
	    //Saving parameter from the camera matrix
	    fl_x_reciprocal = 1.0f / cameraMatrix.at<double>(0,0);
	    fl_y_reciprocal = 1.0f / cameraMatrix.at<double>(1,1);
	    //center_x = _cameraMatrix.at<double>(0,2);
	    //center_y = _cameraMatrix.at<double>(1,2);

	    double noV,
		    apertureWidth = (45/1000)*imgSize.width,
		    apertureHeight = (45/1000)*imgSize.height;
	    cv::calibrationMatrixValues(cameraMatrix, imgSize,
				    apertureWidth, apertureHeight,
				    noV, noV, noV, center, noV);
	} else {
	    BOOST_LOG_TRIVIAL(warning) <<"No cameraMatrix data.";
	    return cv::Point2i();
	}

	cv::Point2i out2Dp;

	out2Dp.x = (point.x/(point.z*fl_x_reciprocal)) + center.x;
	out2Dp.y = (point.y/(point.z*fl_y_reciprocal)) + center.y;
	// Saving found pixel
	return out2Dp;
}

static inline cv::Point3d pointTo3D(cv::Point point, float depthValue)
{
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

	cv::Point3d out3Dp;

	//Saving parameter from the camera matrix
	out3Dp.x = (static_cast<float> (point.x) - center_x) * depthValue * fl_x_reciprocal; //X
	out3Dp.y = (static_cast<float> (point.y) - center_y) * depthValue * fl_y_reciprocal; //Y
	out3Dp.z = depthValue; //Z

	return out3Dp;
}

static inline cv::Point2i pointTo2D(cv::Point3d point)
{
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

	cv::Point2i out2Dp;

	out2Dp.x = (point.x/(point.z*fl_x_reciprocal)) + center_x;
	out2Dp.y = (point.y/(point.z*fl_y_reciprocal)) + center_y;
	// Saving found pixel
	return out2Dp;
}

}}

