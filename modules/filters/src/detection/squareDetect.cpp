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

#include <boost/log/common.hpp>
#include <boost/log/trivial.hpp>
#include <boost/math/special_functions.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "toffy/common/filenodehelper.hpp"
#include "toffy/cam/cameraParams.hpp"

#include "toffy/detection/detectedObject.hpp"
#include "toffy/detection/squareDetect.hpp"

using namespace std;
using namespace cv;

using namespace toffy;
using namespace toffy::detection;

// debug stuff:

std::size_t SquareDetect::_filter_counter = 1;
const std::string SquareDetect::id_name = "squareDetect";

SquareDetect::SquareDetect()
    : Filter(SquareDetect::id_name, _filter_counter),
      in_depth("depth"),
      in_ampl("ampl"),
      in_blobs("objects"),
      out_detect("detection")
{
    _filter_counter++;
}

SquareDetect::~SquareDetect() {}

void SquareDetect::updateConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    in_blobs = pt.get<string>("inputs.blobs", in_blobs);

    boost::optional<const boost::property_tree::ptree&> ocvo =
        pt.get_child_optional("options.cameraMatrix");
    if (ocvo.is_initialized()) {
        if (toffy::commons::checkOCVNone(*ocvo)) {
            boost::property_tree::ptree os;
            os.put_child("cameraMatrix", *ocvo);
            cv::FileStorage fs = commons::loadOCVnode(os);
            fs.getFirstTopLevelNode() >> _cameraMatrix;
            fs.release();
        } else
            BOOST_LOG_TRIVIAL(debug) << "Node cameraMatrix is not opencv.";
    } else
        BOOST_LOG_TRIVIAL(debug) << "Node options.cameraMatrix not found.";

    //minSize = pt.get<int>("options.minSize", minSize);
}

boost::property_tree::ptree SquareDetect::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("inputs.blobs", in_blobs);

    //pt.put("options.minSize", minSize);

    return pt;
}

bool SquareDetect::filter(const Frame& in, Frame& out)
{
    boost::shared_ptr<std::vector<DetectedObject*> > blobs;
    try {
        blobs =
            boost::any_cast<boost::shared_ptr<std::vector<DetectedObject*> > >(
                in.getData(in_blobs));
    } catch (const boost::bad_any_cast&) {
        BOOST_LOG_TRIVIAL(warning)
            << id() << ": Could not find detected object vector:" << in_blobs;
        return false;
    }

    boost::shared_ptr<cv::Mat> outDet;
    try {
        if (out.hasKey(out_detect))
            outDet = boost::any_cast<boost::shared_ptr<cv::Mat> >(
                out.getData(out_detect));
        else {
            outDet.reset(new Mat());
        }
    } catch (const boost::bad_any_cast&) {
        BOOST_LOG_TRIVIAL(warning) << "Could not cast output " << out_detect
                                   << ", filter  " << id() << " not applied.";
        return false;
    }
    if (!cam) {
        if (in.hasKey(CAM_SLOT)) {
            cam = boost::any_cast<cam::CameraPtr>(in.getData(CAM_SLOT));
        } else {
            BOOST_LOG_TRIVIAL(warning) << __FUNCTION__ << " " << id()
                                       << " NO cameraPtr found in Frame!";
            return false;
        }
    }

    boost::shared_ptr<cv::Mat> depth = in.getMatPtr("depth");
    boost::shared_ptr<cv::Mat> depthf = in.getMatPtr("depthf");
    boost::shared_ptr<cv::Mat> floor = in.getMatPtr("floor");

    imshow("depthf square", *depthf);

    Mat imgCopy = Mat::zeros(depthf->size(), CV_8U);

    for (size_t i = 0; i < blobs->size(); i++) {
        if ((*blobs->at(i)->hierarchy)[blobs->at(i)->idx][3] != -1) {
            drawContours(*depthf, *blobs->at(i)->contours, blobs->at(i)->idx,
                         Scalar(0), FILLED, LINE_AA);
            continue;
        }
        drawContours(imgCopy, *blobs->at(i)->contours, blobs->at(i)->idx,
                     Scalar(255), 2, LINE_AA);
    }

    Mat mask = *depthf > 0.30;
    imshow("mask", mask);
    Scalar mean, stddev;
    meanStdDev(*depthf, mean, stddev, mask);
    cout << "mean depth: " << mean.val[0] << endl;
    cout << "stddev depth: " << stddev.val[0] << endl;

    int morph_size = 2;
    Mat kernel = getStructuringElement(
        MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1),
        cv::Point(morph_size, morph_size));
    morphologyEx(imgCopy, imgCopy, MORPH_CLOSE, kernel, Point(-1, -1), 2,
                 BORDER_CONSTANT);

    imshow("imgCopy", imgCopy);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(imgCopy, contours, hierarchy, RETR_EXTERNAL,
                 CHAIN_APPROX_NONE);

    std::cout << contours.size() << std::endl;
    //return true;

    //for( size_t i = 0; i< blobs->size(); i++ )  {
    if (contours.size() > 0) {
        //DetectedObject *b = blobs->at(i);

        vector<Point> approx;
        // Find rectangles from good contours
        approxPolyDP(Mat(contours.front()), approx,
                     arcLength(Mat(contours.front()), true) * 0.001, true);
        //cout << "approx.size(): " << approx.size() << endl;

        // minAreaRect gives us the rotated square. We get the corners from the boundRectangle containing the rotated one.
        cv::RotatedRect rRect = minAreaRect(Mat(approx));

        // Not used because the big distorsion, better leave the code check determine if it is a marker.
        // Checking that the squares detected are squares and not rectangles.
        // We check the difference in pixels of the vertices which are farthest to
        // each other
        //cout << "diff1: " << vertices[0] - vertices[2] << endl;
        //cout << "diff2: " << vertices[1] - vertices[3] << endl;
        /*if ( std::fabs((vertices[0] - vertices[2]).x) > 12 || std::fabs((vertices[1] - vertices[3]).y) > 12) {
	    drawContours(dst, contours, markers[i], Scalar(255,255,255), FILLED, LINE_8);
	    continue;
	}*/

        Mat end = floor->clone();

        //Debug Shows the rectangles
        //Copy the vertice points to a vector
        Point2f vertices[4];
        rRect.points(vertices);
        //cout << "vertices: ";
        for (int i = 0; i < 4; i++) {
            line(*depthf, vertices[i], vertices[(i + 1) % 4],
                 Scalar(255, 255, 255));
            //line(Ishow, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
            //cout << vertices[i] << "; ";
        }
        //cout << endl;

        // Debug Shows bound rectangle
        rectangle(*depthf, rRect.boundingRect(), Scalar(200, 200, 200));
        Mat rot_mat = getRotationMatrix2D(
            Point(depth->cols / 2, depth->rows / 2), rRect.angle, 1);
        warpAffine(end, end, rot_mat, depth->size());

        std::cout << "angles: " << rRect.angle << std::endl;
        std::cout << "center: " << rRect.center << std::endl;
        std::cout << "img size: " << depth->size << std::endl;
        std::cout << "img rows x: " << (depth->rows / 2 - 1) << std::endl;
        std::cout << "img cols y: " << (depth->cols / 2 - 1) << std::endl;
        std::cout << "center shift x: "
                  << fabs((depth->cols / 2 - 1) - rRect.center.x) << std::endl;
        std::cout << "center shift y: "
                  << fabs((depth->rows / 2 - 1) - rRect.center.y) << std::endl;

        Mat trans_mat = (Mat_<double>(2, 3) << 1, 0,
                         fabs((depth->cols / 2 - 1) - rRect.center.x), 0, 1,
                         fabs((depth->rows / 2 - 1) - rRect.center.y));
        warpAffine(end, end, trans_mat, depth->size());

        Mat kiste3Dcorners = (Mat_<Point2f>(1, 4) <<
                                  /*Point2f(0,27.5),
						    Point2f(0,0),
						    Point2f(37.5,0),
						    Point2f(37.5,27.5));*/
                                  Point2f(0, 29),
                              Point2f(0, 0), Point2f(39, 0), Point2f(39, 29));
        Mat kiste3D =
            (Mat_<Point2f> /*(3,4)*/ (1, 20) <<
                 /*Point2f(0,0), Point2f(9.375,0), Point2f(18.75,0), Point2f(28.125,0), Point2f(37.5,0),
		       Point2f(0,9.1666667), Point2f(9.375,9.1666667), Point2f(18.75,9.1666667), Point2f(28.125,9.1666667), Point2f(37.5,9.1666667),
		       Point2f(0,18.3333334), Point2f(9.375,18.3333334), Point2f(18.75,18.3333334), Point2f(28.125,18.3333334), Point2f(37.5,18.3333334),
		       Point2f(0,27.5), Point2f(9.375,27.5), Point2f(18.75,27.5), Point2f(28.125,27.5), Point2f(37.5,27.5));*/
                 Point2f(0, 0),
             Point2f(10, 0), Point2f(20, 0), Point2f(30, 0), Point2f(40, 0),
             Point2f(0, 10), Point2f(10, 10), Point2f(20, 10), Point2f(30, 10),
             Point2f(40, 10), Point2f(0, 20), Point2f(10, 20), Point2f(20, 20),
             Point2f(30, 20), Point2f(40, 20), Point2f(0, 30), Point2f(10, 30),
             Point2f(20, 30), Point2f(30, 30), Point2f(40, 30));

        //Order points anticlockwise with origin in bottom right
        vector<Point2f> s;
        /*s.push_back(rRect.boundingRect().br());
	s.push_back(rRect.boundingRect().tl()+Point(rRect.boundingRect().width,0));
	s.push_back(rRect.boundingRect().tl());
	s.push_back(rRect.boundingRect().tl()+Point(0,rRect.boundingRect().height));*/
        s.push_back(vertices[0]);
        s.push_back(vertices[1]);
        s.push_back(vertices[2]);
        s.push_back(vertices[3]);

        std::cout << "Point bl: " << s[0] << std::endl;
        std::cout << "Point tl: " << s[1] << std::endl;
        std::cout << "Point tr: " << s[2] << std::endl;
        std::cout << "Point br: " << s[3] << std::endl;

        //warpAffine( Mat(s), Mat(s), rot_mat, Mat(s).size() );
        transform(s, s, rot_mat);
        transform(s, s, trans_mat);

        std::cout << "Point after tl: " << s[0] << std::endl;
        std::cout << "Point after bl: " << s[1] << std::endl;
        std::cout << "Point after tr: " << s[2] << std::endl;
        std::cout << "Point after br: " << s[3] << std::endl;

        cv::Point3d l3d, r3d;
        cam->pointTo3D(s[0], 0.36, l3d);
        cam->pointTo3D(s[3], 0.36, r3d);

        cout << "length top: " << (fabs(l3d.x) + fabs(r3d.x)) << endl;

        cam->pointTo3D(s[1], 0.36, l3d);
        cam->pointTo3D(s[2], 0.36, r3d);

        cout << "length bottom: " << (fabs(l3d.x) + fabs(r3d.x)) << endl;

        Point2f lm = (s[0] + s[1]) * .5;
        Point2f rm = (s[2] + s[3]) * .5;

        cout << "lm: " << lm << endl;
        cout << "rm: " << rm << endl;

        cam->pointTo3D(lm, 0.36, l3d);
        cam->pointTo3D(rm, 0.36, r3d);

        cout << "length: " << (fabs(l3d.x) + fabs(r3d.x)) << endl;

        for (size_t i = 0; i < s.size(); i++) {
            line(end, s[i], s[(i + 1) % 4], Scalar(255, 255, 255));
            //cout << vertices[i] << "; ";
        }

        Mat H = findHomography(kiste3Dcorners, s, cv::RANSAC);

        cout << "H: " << H << endl;
        // Project the code points S to the image using H
        std::vector<Point2f> ks;
        std::vector<cv::Point3d> ks3d;
        perspectiveTransform(kiste3D, ks, H);

        // 4x4 identity mtx
        Mat t3D = (Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0.0,
                   0, 0, 0, 1);
        cout << "ks size: " << ks.size() << endl;
        imshow("sq", *depthf);
        warpAffine(*depthf, *depthf, rot_mat, depthf->size());
        warpAffine(*depthf, *depthf, trans_mat, depthf->size());
        for (size_t i = 0; i < ks.size(); i++) {
            //cout << "ks: " << ks[i] << endl;
            //circle( end, ks[i], 2, Scalar(255), FILLED, LINE_8 );
            cv::Point3d pt;
            cam->pointTo3D(ks[i], mean.val[0], pt);
            ks3d.push_back(pt);
            // ks3d.push_back(toffy::commons::pointTo3D(ks[i], /*0.365*/mean.val[0], _cameraMatrix, depth->size()));
            //cout << "ks3d: " << ks3d[i] << endl;
            //Mat src = (cv::Mat_<double>(4,1) << ks3d[i].x, ks3d[i].y, ks3d[i].z, 1.0);
            Mat b = t3D * (cv::Mat_<double>(4, 1) << ks3d[i].x, ks3d[i].y,
                           ks3d[i].z, 1.0);
            //cout << "b: " << b << endl;
            ks3d[i] =
                cv::Point3d(b.at<double>(0), b.at<double>(1), b.at<double>(2));
            //cout << "ks3d: " << ks3d[i] << endl;
            // (t3D * Mat(ks3d[i])).copyTo(b);

            // project back:
            Point2i pi;
            cam->pointTo2D(ks3d[i],pi);
            ks[i] = pi;
            //cout << "ks: " << ks[i] << endl;

            circle(*depthf, ks[i], 2, Scalar(255), FILLED, LINE_8);

            //imshow("end", end);
            //waitKey(0);
        }

        imshow("end", end);

        imshow("sq2", *depthf);
    }

    //out.addData(out_detect, outDet);

    return true;
}
