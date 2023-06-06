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
#include <boost/log/trivial.hpp>
#include <limits.h>

#if OCV_VERSION_MAJOR >= 3
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#else
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/angles.h>

#include "toffy/3d/groundprojection.hpp"
#include "toffy/common/filenodehelper.hpp"

using namespace toffy;
using namespace toffy::filters::f3d;
using namespace cv;
using namespace std;

#ifdef CM_DEBUG
const bool dbg = true;
#else
const bool dbg = false;
#endif

std::size_t GroundProjection::_filter_counter = 1;
const std::string GroundProjection::id_name = "groundprojection";

GroundProjection::GroundProjection()
    : Filter(GroundProjection::id_name, _filter_counter),
      _in_cloud("cloud"),
      _in_img("ground"),
      _out_img("ground"),
      _max_y(0),
      _max_x(0),
      _scale(100),
      _fovx(90.),
      _fovy(67.5),
      _dis(0.0),
      _projBack(false)
{
    _filter_counter++;
}

void GroundProjection::updateConfig(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    boost::optional<const boost::property_tree::ptree &> ocvo =
        pt.get_child_optional("options.cameraMatrix");
    // cout << "options.cameraMatrix: " <<
    // pt.get<string>("options.cameraMatrix") << endl; cout <<
    // "options.distCoeffs: " << pt.get<string>("options.distCoeffs") << endl;
    // cout << "options.fovx: " << pt.get<string>("options.fovx") << endl;
    // cout << "options.fovy: " << pt.get<string>("options.fovy") << endl;
    // cout << "options.dis: " << pt.get<string>("options.dis") << endl;

    if (ocvo.is_initialized()) {
        if (toffy::commons::checkOCVNone(*ocvo)) {
            boost::property_tree::ptree os;
            os.put_child("cameraMatrix", *ocvo);
            cv::FileStorage fs = commons::loadOCVnode(os);
            cout << fs.getFirstTopLevelNode().name() << endl;
            fs.getFirstTopLevelNode() >> _cameraMatrix;
            fs.release();
        } else
            BOOST_LOG_TRIVIAL(debug) << "Node cameraMatrix is not opencv.";
    } else
        BOOST_LOG_TRIVIAL(debug) << "Node options.cameraMatrix not found.";

    _max_x = pt.get<float>("options.max_x", _max_x);
    _max_y = pt.get<float>("options.max_y", _max_y);
    _scale = pt.get<int>("options.scale", _scale);
    _fovx = pt.get<double>("options.fovx", _fovx);
    _fovy = pt.get<double>("options.fovy", _fovy);
    _dis = pt.get<double>("options.dis", _dis);
    _projBack = pt.get<bool>("options.projBack", _projBack);

    _in_cloud = pt.get<string>("inputs.cloud", _in_cloud);
    _in_img = pt.get<string>("inputs.img", _in_img);
    _out_img = pt.get<string>("outputs.img", _out_img);
}

boost::property_tree::ptree GroundProjection::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.max_x", _max_x);
    pt.put("options.max_y", _max_y);
    pt.put("options.scale", _scale);
    pt.put("options.fovx", _fovx);
    pt.put("options.fovy", _fovy);
    pt.put("options.dis", _dis);
    pt.put("options.projBack", _projBack);

    pt.put("inputs.cloud", _in_cloud);
    pt.put("inputs.img", _in_img);
    pt.put("outputs.img", _out_img);

    return pt;
}

bool GroundProjection::filter(const Frame &in, Frame &out)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    double maxSizeX, maxSizeY, fl_x_reciprocal = 1., fl_y_reciprocal = 1.;
    Point2d center;
    Size imgSize(160, 120);

    if (_cameraMatrix.data) {
        // Saving parameter from the camera matrix
        fl_x_reciprocal = 1.0f / _cameraMatrix.at<double>(0, 0);
        fl_y_reciprocal = 1.0f / _cameraMatrix.at<double>(1, 1);
        // center_x = _cameraMatrix.at<double>(0,2);
        // center_y = _cameraMatrix.at<double>(1,2);

        double noV, apertureWidth = (45 / 1000) * imgSize.width,
                    apertureHeight = (45 / 1000) * imgSize.height;
        calibrationMatrixValues(_cameraMatrix, imgSize, apertureWidth,
                                apertureHeight, _fovx, _fovy, noV, center, noV);
    }

    if (_projBack == false) {
        // Doing projection to ground
        cout << "Doing projection to ground." << endl;

        // Getting cloud
        pcl::PCLPointCloud2Ptr cloud;
        try {
            cloud =
                boost::any_cast<pcl::PCLPointCloud2Ptr>(in.getData(_in_cloud));
        } catch (const boost::bad_any_cast &) {
            BOOST_LOG_TRIVIAL(warning)
                << "Could not cast input " << _in_cloud << ", filter  " << id()
                << " not applied.";
            return false;
        }

        // Calculating the size of the view
        if (_dis > 0.0) {
            maxSizeX = (tan(pcl::deg2rad(_fovx) / 2) * _dis * 2) * _scale;
            maxSizeY = (tan(pcl::deg2rad(_fovy) / 2) * _dis * 2) * _scale;
            cout << "maxSizeX: " << maxSizeX << endl;
            cout << "maxSizeY: " << maxSizeY << endl;
        } else {
            if (_max_x <= 0 || _max_y <= 0) {
                BOOST_LOG_TRIVIAL(warning)
                    << "Wrong max values, x=" << _max_x << ", y=" << _max_y
                    << ", filter  " << id() << " not applied.";
                return false;
            }
            maxSizeX = _max_x * _scale;
            maxSizeY = _max_y * _scale;
        }

        cout << "Sizes maxSizeX:" << maxSizeX << " maxSizeY: " << maxSizeY
             << endl;

        // Getting the output if exist and resizing if needed.
        boost::shared_ptr<cv::Mat> proj2d;
        try {
            proj2d = in.getMatPtr(_out_img);
            cout << "proj2d found. " << proj2d->size() << endl;
        } catch (const boost::bad_any_cast &) {
            BOOST_LOG_TRIVIAL(warning)
                << "Could not cast output " << _out_img << ", filter  " << id();
            proj2d.reset(new Mat(maxSizeY, maxSizeX, CV_32F,
                                 numeric_limits<float>::quiet_NaN()));
            out.addData(_out_img, proj2d);
        }

        BOOST_LOG_TRIVIAL(debug) << "Sizes: " << proj2d->size()
                                 << " ==  " << Size(maxSizeX, maxSizeY);
        if (!proj2d->data || proj2d->size() != Size(maxSizeX, maxSizeY)) {
            BOOST_LOG_TRIVIAL(debug) << "Output " << _out_img
                                     << " found but empty of different size."
                                     << " Allocating data. Filter " << id();
            proj2d->release();
            proj2d.reset(new Mat(maxSizeY, maxSizeX, CV_32F,
                                 numeric_limits<float>::quiet_NaN()));
            out.addData(_out_img, proj2d);
        } else
            *proj2d = numeric_limits<float>::quiet_NaN();

        pcl::PointCloud<pcl::PointWithRange>::Ptr cloudXYZ(
            new pcl::PointCloud<pcl::PointWithRange>);
        pcl::fromPCLPointCloud2(*cloud, *cloudXYZ);

        static pcl::PointWithRange p;
        // cout << "cloud->size()" << cloudXYZ->size() << endl;
        for (size_t i = 0; i < cloudXYZ->size(); ++i) {
            p = cloudXYZ->points[i];
            if (p.z != p.z) continue;
            // Go for cm
            int valX = p.x * _scale;
            int valY = p.y * _scale;
            // Shift the values to be all positive
            valX += maxSizeX / 2;
            valY += maxSizeY / 2;

            // Invalid pixels could make the cm coordinates goes
            // beyond the sizes.
            if (valX < 0 || valY < 0 || valX >= maxSizeX || valY >= maxSizeY) {
                cout << "(" << valX << "," << valY << ")" << endl;
                continue;
            }

            proj2d->at<float>(valY, valX) = p.range;

            /*double yc = p.y * _scale;
            double xc = p.x * _scale;
            if (p.y > maxSizeY || p.x > maxSizeX)
                continue;

            double sizex = p.range * factx,
                    sizey = p.range * facty;

            for (int x= xc-sizex/2; x<xc+(sizex+1)/2; x++) {
                for (int y= yc-sizey/2; y<yc+(sizey+1)/2; y++) {
                    if (_dis > 0.0) {
                        x += maxSizeX/2;
                        y += maxSizeY/2;
                    }

                    // skip if we're out
                    if (x<0 || y<0
                            || x >= proj2d->cols
                            || y >= proj2d->rows)
                        continue;

                    if (proj2d->at<float>(y,x) != proj2d->at<float>(y,x))
                        proj2d->at<float>(y,x) = p.z;
                    else {
                        proj2d->at<float>(y,x) += p.z;
                        proj2d->at<float>(y,x) /= 2;
                    }
                }
            }*/
        }
        imshow("ground insider", *proj2d);
        cout << "proj2d: " << proj2d->size() << endl;
    } else {
        // Projecting back to 2d from ground
        cout << "Projecting back to 2d from ground." << endl;

        boost::shared_ptr<cv::Mat> fground;
        try {
            fground = in.getMatPtr(_in_img);
        } catch (const boost::bad_any_cast &) {
            BOOST_LOG_TRIVIAL(error) << "Could not cast output " << _in_img
                                     << ", filter  " << id() << " not applied.";
            return false;
        }

        boost::shared_ptr<cv::Mat> pback;
        try {
            pback = in.getMatPtr(_out_img);
        } catch (const boost::bad_any_cast &) {
            BOOST_LOG_TRIVIAL(debug) << "Could not cast output " << _out_img;
            // TODO were is the size???
            pback.reset(new cv::Mat(Mat::zeros(imgSize, CV_32F)));
            out.addData(_out_img, pback);
        }
        *pback = 0.0;

        maxSizeX = (tan(pcl::deg2rad(_fovx) / 2) * _dis * 2);
        maxSizeY = (tan(pcl::deg2rad(_fovy) / 2) * _dis * 2);

        cout << "maxSizeX " << maxSizeX << endl;
        cout << "maxSizeY " << maxSizeY << endl;
        cout << "fground: " << fground->size() << endl;

        for (int y = 0; y < fground->rows; ++y) {
            for (int x = 0; x < fground->cols; ++x) {
                // Avoing Z NaN values

                if (fground->at<float>(y, x) != fground->at<float>(y, x)) {
                    continue;
                }
                cout << "xy " << x << "/" << y << endl;
                double fValX = x / (_scale * 1.f);
                double fValY = y / (_scale * 1.f);
                cout << "fground->at<float>(y,x) " << fground->at<float>(y, x)
                     << endl;
                // maxSizeX =
                // (tan(pcl::deg2rad(_fovx)/2)*fground->at<float>(y,x)*2)*_scale;
                // maxSizeY =
                // (tan(pcl::deg2rad(_fovy)/2)*fground->at<float>(y,x)*2)*_scale;
                // cout << "maxSizeX " << maxSizeX << endl;
                // cout << "maxSizeY " << maxSizeY << endl;
                cout << "noscale " << fValX << "/" << fValY << endl;
                fValX = fValX - maxSizeX / 2;
                fValY = fValY - maxSizeY / 2;
                cout << "noscale2 " << fValX << "/" << fValY << endl;
                // Projecting back to image 2D coordinates
                // int iValX = (x*160)/maxSizeX;
                // int iValY = (y*120)/maxSizeY;

                // Projecting back to image 2D coordinates
                int iValX =
                    fValX / (fground->at<float>(y, x) * fl_x_reciprocal);
                int iValY =
                    fValY / (fground->at<float>(y, x) * fl_y_reciprocal);

                cout << "to pixels before " << iValX << "/" << iValY << endl;
                iValX += center.x;
                iValY += center.y;

                // Return to relative to camera positions (also negatives)
                // cout << "noscale " << fValX << "/" << fValY << endl;
                // int iValX = (fValX/(fground->at<float>(y,x)*factx)) + 79;
                // int iValY = (fValY/(fground->at<float>(y,x)*facty)) + 59;
                cout << "to pixels " << iValX << "/" << iValY << endl;

                // Saving found pixel
                if (iValX >= 0 && iValX < pback->cols && iValY >= 0 &&
                    iValY < pback->rows)
                    pback->at<float>(iValY, iValX) = fground->at<float>(y, x);
                else
                    cout << "ERR " << iValX << "/" << iValY << endl;
            }
        }
        imshow("pback insider", *pback);
    }

    return true;
}

/*
void GroundProjection::projectBack(const cv::Mat &mask, double maxSizeX , double
maxSizeY) { BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    double factx = sin( pcl::deg2rad(_fovx)),
            facty = sin( pcl::deg2rad(_fovy));

    for (int y=0; y < proj2d.rows; ++y) {
        for (int x=0; x < proj2d.cols; ++x) {
            // Avoing Z NaN values
            if (proj2d.at<float>(y,x) != proj2d.at<float>(y,x)) {
                    continue;
            }
            //Return to relative to camera positions (also negatives)
            float fValX = x - maxSizeX/2;
            float fValY = y - maxSizeY/2;

            fValX /= factx;
            fValY /= facty;
            // Back from cm to meters.
            int iValX = fValX / _scale;
            int iValY = fValY / _scale;

            / *if (mask.at<float>(fValX,fValY) != mask.at<float>(fValX,fValY) )
{

            } else {

            }* /

            //Projecting back to image 2D coordinates
            //int iValX = (fValX/(proj2d.at<float>(y,x)*fl_x_reciprocal)) +
center_x;
            //int iValY = (fValY/(proj2d.at<float>(y,x)*fl_y_reciprocal)) +
center_y;
            // Saving found pixel

            if ( iValX >= 0 && iValX < mask.cols
                 && iValY >= 0 && iValY < mask.rows )
                    mask.at<unsigned char>(iValY, iValX) = 255;
            else
                    cout << "ERR " << iValX << "/" << iValY << endl;


        }
    }
}*/
