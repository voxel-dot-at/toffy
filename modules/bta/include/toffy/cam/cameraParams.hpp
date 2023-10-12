/*
   Copyright 2012-2021 Simon Vogl <svogl@voxel.at> VoXel Interaction Design -
   www.voxel.at

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
#pragma once
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

#include <boost/log/trivial.hpp>

#include <opencv2/calib3d.hpp>

namespace toffy {
namespace cam {

/// name for the key in the frame
#define CAM_SLOT "camera"


/** helper functions for camera handling; assumes we have standard optics (90deg
 * hfov).
 */

static float deg2rad(float a) { return a / 180. * M_PI; }

/**
 * @brief Camera geometry meta-data 
 * 
 */
class Camera
{
   public:
    std::string name;
    cv::Mat cameraMatrix;
    cv::Size imgSize;
    cv::Point2d center;

    double apertureWidth, apertureHeight;  // width/height of the sensor in mm
    double fl_x_reciprocal, fl_y_reciprocal;

    float pixSinAngle = 0.009817319;  // = sin( deg2rad(90/160) );
    float pixCosAngle = 0.999951809;  // = cos( deg2rad(90/160) );

    Camera(const std::string& camName, const cv::Size& size, double aperWidth, double aperHeight)
        : name(camName), imgSize(size), apertureWidth(aperWidth), apertureHeight(aperHeight)
    {
    }

    Camera(const std::string& camName, const cv::Size& size, double aperWidth, double aperHeight,
           const cv::Mat& matrix)
        : name(camName), cameraMatrix(matrix),
          imgSize(size),
          apertureWidth(aperWidth),
          apertureHeight(aperHeight)
    {
        init(matrix);
    }
    /**
     * @brief init camera matrix and helper variables
     *
     * @param matrix the 3x3 double values camera matrix
     */
    void init(const cv::Mat& matrix)
    {
        cameraMatrix = matrix;
        fl_x_reciprocal = 1.0f / cameraMatrix.at<double>(0, 0);
        fl_y_reciprocal = 1.0f / cameraMatrix.at<double>(1, 1);

        // unused values:
        double fovX, fovY, focalLength, aspectRatio;
        cv::calibrationMatrixValues(matrix, imgSize, apertureWidth,
                                    apertureHeight, fovX, fovY, focalLength,
                                    center, aspectRatio);
    }

    inline void pointTo2D(const cv::Point3d& point, cv::Point2i& out)
    {
        out.x = (point.x / (point.z * fl_x_reciprocal)) + center.x;
        out.y = (point.y / (point.z * fl_y_reciprocal)) + center.y;
    }

    inline void pointTo3D(const cv::Point2d& point, float depthValue,
                          cv::Point3d& out)
    {
        // BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << point;

        // Saving parameter from the camera matrix
        out.x = (static_cast<float>(point.x) - center.x) * depthValue *
                fl_x_reciprocal;
        out.y = (static_cast<float>(point.y) - center.y) * depthValue *
                fl_y_reciprocal;
        out.z = depthValue;
    }
    double getPixelSize(double distance)
    {
        // float maxFinger = 0.01 / (angl*pdis);  //==pixel width
        return pixSinAngle * distance;
    }

};

typedef boost::shared_ptr<Camera> CameraPtr;

class P230 : public Camera  // it's a cif chip
{
    /**

Infos der P230:
FoV:
HorizontalFOV [°]=79,75
VerticalFOV [°]=66,88
Auflösung:
352x287 Pixel
     * 
     */
   public:
    // 90 degrees hvof, 352x287px sensor resolution, 17.5u pixel size
    P230()
        : Camera("p230", cv::Size(352, 287), (17.5 / 1000.) * 352, (17.5 / 1000.) * 288)
    {
        cv::Mat mtx = (cv::Mat_<double>(3, 3) << 8.8345892962843834e+01, 0.,
                       7.9460485484676596e+01, 0., 8.8395902341635306e+01,
                       5.7816728185872989e+01, 0., 0., 1.);
        init(mtx);
        pixSinAngle = 0.004462475;  // = sin( deg2rad(90/352) );
        pixCosAngle = 0.999990043;  // = cos( deg2rad(90/352) );
    }
};

class M520 : public Camera  // also M100, P100, ...
{
   public:
    // 90 degrees hvof, 160x120px sensor resolution
    M520() : Camera("m520", cv::Size(160, 120), (45 / 1000.) * 160, (45 / 1000.) * 120)
    {
        cv::Mat mtx =
            (cv::Mat_<double>(3, 3) << 8.8345892962843834e+01, 0.,
             7.9460485484676596e+01,                              /// 88  0 79
             0., 8.8395902341635306e+01, 5.7816728185872989e+01,  ///  0 88 59
             0., 0., 1.);                                         ///  0  0  1
        init(mtx);
        // TODO: actually compute from camera matrix
        pixSinAngle = 0.009817319;  // = sin( deg2rad(90/160) );
        pixCosAngle = 0.999951809;  // = cos( deg2rad(90/160) );
    }

    const int hPix = 160;
    const int vPix = 120;
    const float hFov = 90.;
    const float vFov = 65.;
    // const float pixSinAngle = 0.009817319;  // = sin( deg2rad(90/160) );
    // const float pixCosAngle = 0.999951809;  // = cos( deg2rad(90/160) );
    // const double fl_x_reciprocal = 1.0f / 8.8345892962843834e+01;
    // const double fl_y_reciprocal = 1.0f / 8.8395902341635306e+01;
    const double center_x = 7.9460485484676596e+01;
    const double center_y = 5.7816728185872989e+01;

    inline void depth2xyz(const cv::Point2f& p, float d, cv::Point3f& xyz)
    {
        xyz.x =
            pixSinAngle * (p.x - hPix / 2) * pixSinAngle * (p.y - vPix / 2) * d;
        xyz.y =
            pixSinAngle * (p.x - hPix / 2) * pixCosAngle * (p.y - vPix / 2) * d;
        xyz.z = pixCosAngle * (p.x - hPix / 2) * d;

        float rho = deg2rad((p.x - hPix / 2));
        float tht = deg2rad((p.y - vPix / 2));

        xyz.x = d * sin(rho) * sin(tht);
        xyz.y = d * sin(rho) * cos(tht);
        xyz.z = d * cos(rho);
    }

    inline void depth2xyz(const cv::Point2f& p, float d, cv::Vec3f& xyz)
    {
        xyz[0] =
            pixSinAngle * (p.x - hPix / 2) * pixSinAngle * (p.y - vPix / 2) * d;
        xyz[1] =
            pixSinAngle * (p.x - hPix / 2) * pixCosAngle * (p.y - vPix / 2) * d;
        xyz[2] = pixCosAngle * (p.x - hPix / 2) * d;
    }
};

}  // namespace cam
};  // namespace toffy
