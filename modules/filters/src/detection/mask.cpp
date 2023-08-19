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
#include <math.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <boost/log/trivial.hpp>
#include <toffy/common/filenodehelper.hpp>

#include <toffy/detection/mask.hpp>

using namespace toffy;
using namespace toffy::detection;
//using namespace cv;
using namespace std;


std::size_t Mask::_filter_counter = 1;
const std::string Mask::id_name = "mask";

Mask::Mask(): Filter(Mask::id_name,_filter_counter),
    _in_depth("depth"), _in_ampl("ampl"), _out_mask("mask")
{
    _filter_counter++;

}

boost::property_tree::ptree Mask::getConfig() const {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();
    //pt.put("options.max_x", _max_x);

    pt.put("options.maskPath", _maskPath);
    pt.put("options.creation", _creation);
    pt.put("options.dis", _dis);
    pt.put("options.scale", _scale);
    //pt.put("options.cameraMatrix", _cameraMatrix);

    pt.put("inputs.depth", _in_depth);
    pt.put("inputs.ampl", _in_ampl);
    pt.put("outputs.mask", _out_mask);

    return pt;
}

void Mask::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    //_max_x = pt.get<float>("options.max_x",_max_x);
    _maskPath = pt.get<string>("options.maskPath",_maskPath);
    _creation = pt.get<bool>("options.creation", _creation);
    _creation = pt.get<bool>("options.creation",false);
    _dis = pt.get<double>("options.dis",_dis);
    _scale = pt.get<int>("options.scale",_scale);

    boost::optional<const boost::property_tree::ptree& > ocvo =
            pt.get_child_optional( "options.cameraMatrix" );
    if (ocvo.is_initialized()) {
        if( toffy::commons::checkOCVNone(*ocvo) ) {
            boost::property_tree::ptree os;
            os.put_child("cameraMatrix",*ocvo);
            cv::FileStorage fs = commons::loadOCVnode(os);
            //cout << fs.getFirstTopLevelNode().name() << endl;
            if (!_cameraMatrix)
                _cameraMatrix.reset(new cv::Mat());
            fs.getFirstTopLevelNode() >> *_cameraMatrix;
            fs.release();
        } else BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
        BOOST_LOG_TRIVIAL(debug) << "Node cameraMatrix is not opencv.";
    } else
        BOOST_LOG_TRIVIAL(debug) << "Node options.cameraMatrix not found.";

    _in_depth = pt.get<string>("inputs.depth",_in_depth);
    _in_ampl = pt.get<string>("inputs.ampl",_in_ampl);
    _out_mask = pt.get<string>("outputs.mask",_out_mask);
}

bool Mask::filter(const Frame& in, Frame& out) {

    // General input: depth image
    try {
        depth = in.getMatPtr(_in_depth);
    } catch(const boost::bad_any_cast &) {
        BOOST_LOG_TRIVIAL(warning) << "Could not cast input " << _in_depth <<
                                      ", filter  " << id() <<" not applied.";
        return false;
    }

    if (_creation) {

        // Creation input: Ampl image, Camera matrix
        try {
            ampl = in.getMatPtr(_in_ampl);
        } catch(const boost::bad_any_cast &) {
            BOOST_LOG_TRIVIAL(warning) <<
                                          "Could not cast input " << _in_ampl <<
                                          ", filter  " << id() <<" not applied.";
            return false;
        }

        if (!_cameraMatrix || _cameraMatrix->empty()) {
            BOOST_LOG_TRIVIAL(warning) << "No cameraMatrix data, filter "
                                       << id() <<" not applied.";
            return false;
        }


        static double focalLength, aspectRatio;
        static cv::Point2d center;
        _apertureWidth = 6.16;
        _apertureHeight = 4.62;
        cv::calibrationMatrixValues(*_cameraMatrix, depth->size(),
                                    _apertureWidth, _apertureHeight,
                                    fovx, fovy,
                                    focalLength, center, aspectRatio);


        generate3D();

        //groundProjection();
        groundProjection2();
        if (dbg) {
            cv::namedWindow(id_name + "[1] proj2d", cv::WINDOW_NORMAL);
            cv::imshow(id_name + "[1] proj2d", *proj2d);
        }

        if (!fground) {
            fground.reset(new cv::Mat());
        }
        *fground = proj2d->clone();
        BOOST_LOG_TRIVIAL(debug) << "fground->size()" << fground->size();
        //cout << "fground->size()" << fground->size() << endl;
        /*
        for(int i = 0; i < proj2d->rows; i++) {
            const float* ptr = proj2d->ptr<float>(i);
            for(int j = 0; j < proj2d->cols; j++) {
                if(ptr[j] == ptr[j]) {
                    cv::rectangle(*fground, cv::Point (j-maxSizeY/2,i-maxSizeX/2),
                              cv::Point (j+maxSizeY/2,i+maxSizeX/2), ptr[j],
                              cv::FILLED);
                }
            }
        }
        if (dbg) cv::imshow("fground", *fground);
        */
        //Project back
        projectBack();
        if (dbg) {
            cv::namedWindow(id_name + "[2] projectBack", cv::WINDOW_NORMAL);
            imshow(id_name + "[2] projectBack", *new_mask);
        }

        //Add low amp pixel to mask
        /*amplitudeMasking();
        if (dbg) {
            cv::namedWindow("amplitudeMasking", cv::WINDOW_NORMAL);
            imshow("*amplitudeMasking", *new_mask);
        }*/

        morfExMask(*new_mask);
        if (dbg) {
            cv::namedWindow(id_name + "[3] morfExMask", cv::WINDOW_NORMAL);
            imshow(id_name + "[3] morfExMask", *new_mask);
        }


        fillMaskGaps();
        if (dbg) {
            cv::namedWindow(id_name + "[4] new_mask insider", cv::WINDOW_NORMAL);
            imshow(id_name + "[4] new_mask insider", *new_mask);
        }

        *new_mask=~*new_mask;

        cv::Mat add_depth, fdepth;
        depth->copyTo(fdepth, *new_mask);
        add_depth = fdepth > 0;
        bitwise_xor(*new_mask,add_depth,*new_mask);

        boost::shared_ptr<cv::Mat> old_mask;
        try {
            old_mask = in.getMatPtr(_out_mask);
        } catch(const boost::bad_any_cast &) {
            BOOST_LOG_TRIVIAL(debug) << "Could not cast previous mask.";
            out.addData(_out_mask,new_mask);
            old_mask = new_mask;
        }
        *old_mask &= *new_mask;

        if (dbg) {
            cv::namedWindow(id_name + "[5] mask", cv::WINDOW_NORMAL);
            imshow(id_name + "[5] mask", *old_mask);
        }

        boost::shared_ptr<cv::Mat> depth_mask;
        if(!depth_mask) {
            depth_mask.reset(new cv::Mat());
            depth->copyTo(*depth_mask,~*old_mask);
            //cv::subtract(*depth_mask,*depth_mask*(5./100),*depth_mask);
            *depth_mask -= *depth_mask*(5./100);
        }

        cv::Mat temp;
        depth->copyTo(temp,~*old_mask);
        *depth_mask += (temp - temp*(5./100));
        *depth_mask /= 2.f;

        if (dbg) {
            cv::namedWindow("1depth_mask", cv::WINDOW_NORMAL);
            imshow("1depth_mask", *depth_mask);
        }

        BOOST_LOG_TRIVIAL(debug) << "depth_mask nomorph 20x77: " << depth_mask->at<float>(77,20);
        int morph_size = 1;
        cv::Mat structuringElement = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size( 2*morph_size + 1, 2*morph_size+1 ),
                    cv::Point( morph_size, morph_size ));

        cv::morphologyEx( *depth_mask, *depth_mask,
                          cv::MORPH_CLOSE, structuringElement, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT );
        //morfExMask(*depth_mask);
        //*depth_mask = ~*depth_mask;
        cv::morphologyEx( *depth_mask, *depth_mask,
                          cv::MORPH_CLOSE, structuringElement, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT );

        BOOST_LOG_TRIVIAL(debug) << "depth 20x77: " << depth->at<float>(77,20);
        BOOST_LOG_TRIVIAL(debug) << "depth_mask 20x77: " << depth_mask->at<float>(77,20);

        cv::FileStorage fs(_maskPath+"/mask.yml", cv::FileStorage::WRITE);
        fs << "mask" << *depth_mask;

        if (dbg) {
            cv::namedWindow("2depth_mask", cv::WINDOW_NORMAL);
            imshow("2depth_mask", *depth_mask);
        }

        //fdepth = 0;
        //depth->copyTo(fdepth, *mask);
        depth->copyTo(fdepth);

        BOOST_LOG_TRIVIAL(debug) << "fdepth 20x77: " << fdepth.at<float>(77,20);
        BOOST_LOG_TRIVIAL(debug) << "depth_mask 20x77: " << depth_mask->at<float>(77,20);

        cv::Mat m = (fdepth > *depth_mask) & (*depth_mask > 0.);

        if (dbg) {
            cv::namedWindow("comp", cv::WINDOW_NORMAL);
            imshow("comp", m);
        }

        fdepth.setTo(0., m);

        //*depth &= *mask;
        if (dbg) {
            cv::namedWindow("depthmask masked", cv::WINDOW_NORMAL);
            imshow("depthmask masked", fdepth);
        }
        *depth = fdepth;

        //save it
        vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(0);
        cv::imwrite( _maskPath+"/test.png", *old_mask,compression_params );

        fdepth.setTo(0.);
        depth->copyTo(fdepth, *old_mask);
        //*depth &= *mask;
        if (dbg) {
            cv::namedWindow("depth masked", cv::WINDOW_NORMAL);
            imshow("depth masked", fdepth);
        }
        *depth = fdepth;

        //Time/Angle mask creation control
        /*if(_uangle < 0) {
            //First mask
            out.addData(_out_mask,new_mask);
            //_uangle = new_ang;

        } else if (_uangle == new_ang) {
            //Same mask, do and op
            boost::shared_ptr<cv::Mat> old_mask;
            try {
                old_mask = in.getMatPtr(_out_mask);
            } catch(const boost::bad_any_cast &) {
                BOOST_LOG_TRIVIAL(debug) <<
                                            "Could not cast previous mask.";
                out.addData(_out_mask,new_mask);
                old_mask = new_mask;
            }
            *old_mask &= *new_mask;

            //save it
            vector<int> compression_params;
            compression_params.push_back(IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(0);
            cv::imwrite( "test"+boost::lexical_cast<std::string>(_uangle)+".png", *old_mask,compression_params );
        }
        _uangle = new_ang;*/

    } else {

        //Load depth and ampl angle by side
        BOOST_LOG_TRIVIAL(debug) << "Loading mask.";
        boost::shared_ptr<cv::Mat> mask;
        try {
            mask = in.getMatPtr(_out_mask);
        } catch(const boost::bad_any_cast &) {
            BOOST_LOG_TRIVIAL(debug) << "Not mask.";
            mask.reset(new cv::Mat());
            out.addData("mask", mask);
        }

        if (mask->empty()) {
            //Load mask;
            BOOST_LOG_TRIVIAL(debug) << "New angle, loading.";
            *mask = cv::imread(_maskPath+"/test.png",cv::IMREAD_UNCHANGED);
            if (mask->data == NULL) {
                BOOST_LOG_TRIVIAL(warning) << "Could not load mask file: " <<
                                              _maskPath+"/test"+".png" <<
                                              ", filter  " << id();
            }
        }
        if (dbg) {
            cv::namedWindow("Mask", cv::WINDOW_NORMAL);
            imshow("Mask", *mask);
        }

        cv::Mat dmask;
        cv::FileStorage fs(_maskPath+"/mask.yml", cv::FileStorage::READ);
        fs["mask"] >> dmask;

        if (dbg) {
            cv::namedWindow("dmask", cv::WINDOW_NORMAL);
            imshow("dmask", dmask);
        }

        cv::Mat fdepth;
        //depth->copyTo(fdepth, *mask);
        depth->copyTo(fdepth);

        BOOST_LOG_TRIVIAL(debug) << "fdepth 20x77: " << fdepth.at<float>(77,20);
        BOOST_LOG_TRIVIAL(debug) << "dmask 20x77: " << dmask.at<float>(77,20);

        cv::Mat m = (fdepth > dmask) & (dmask > 0.);

        if (dbg) {
            cv::namedWindow("comp", cv::WINDOW_NORMAL);
            imshow("comp", m);
        }

        fdepth.setTo(0., m);

        //*depth &= *mask;
        if (dbg) {
            cv::namedWindow("depth masked", cv::WINDOW_NORMAL);
            imshow("depth masked", fdepth);
        }
        *depth = fdepth;
    }

    return true;
}

static inline double deg2rad(float degrees ) {
    return (degrees*M_PI)/180.0;
}

void Mask::groundProjection2() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    cv::Mat channel[3];

    //Get every single axis values and his min and max
    cv::split(*img3d, channel);

    //maxSizeX = (tan(pcl::deg2rad(fovx)/2)*_dis*2)*_scale;
    //maxSizeY = (tan(pcl::deg2rad(fovy)/2)*_dis*2)*_scale;
    maxSizeX = (tan(deg2rad(fovx)/2)*_dis*2)*_scale;
    maxSizeY = (tan(deg2rad(fovy)/2)*_dis*2)*_scale;

    //BOOST_LOG_TRIVIAL(debug) << "maxSizeX: " << maxSizeX;
    //BOOST_LOG_TRIVIAL(debug) << "maxSizeY: " << maxSizeY;

    if (!proj2d || !proj2d->data || proj2d->size() != cv::Size(maxSizeX,maxSizeY)) {
        proj2d.reset(new cv::Mat(maxSizeY, maxSizeX, CV_32F,
                                 numeric_limits<float>::quiet_NaN ()));
    } else
        *proj2d = numeric_limits<float>::quiet_NaN ();

    //cout << "planar->size()" << planar->size() << endl;
    //cout << "proj2d->size()" << proj2d->size() << endl;
    //cout << "img3d->size()" << img3d->size() << endl;

    // For each 3D point maps the coordinates.
    for (int y=0; y < img3d->rows; ++y) {
        for (int x=0; x < img3d->cols; ++x) {
            //cout << "(" << x << "," << y << ")" << endl;
            // Avoing Z NaN values
            if (channel[2].at<float>(y,x) != channel[2].at<float>(y,x))												//test.at<float>(y,x) = numeric_limits<double>::quiet_NaN ();
                continue;
            //TODO PARAMETERS!!
            if (channel[2].at<float>(y,x) > _dis || channel[2].at<float>(y,x) < _dis*0.05)
                continue;
            // Go for cm
            int valX = channel[0].at<float>(y,x)*_scale;
            int valY = channel[1].at<float>(y,x)*_scale;
            //Shift the values to be all positive
            valX += maxSizeX/2;
            valY += maxSizeY/2;

            // Invalid pixels could make the cm coordinates goes
            // beyond the sizes.
            if (valX < 0 || valY < 0 || valX >= (int)maxSizeX || valY >= (int)maxSizeY) {
                //cout << "(" << valX << "," << valY << ")" << endl;
                continue;
            }
            //cout << "(" << valX << "," << valY << ")" << endl;
            proj2d->at<float>(valY,valX) = channel[2].at<float>(y,x);
        }
    }
    //TEST
    //channel[2].copyTo(*depth);
}

void Mask::projectBack() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    if (!new_mask || !new_mask->data || new_mask->size() != depth->size()) {
        new_mask.reset(new cv::Mat(cv::Mat::zeros(depth->size(), CV_8U)));
    } else
	new_mask->setTo(0);

    //Project back
    maxSizeX /= _scale;
    maxSizeY /= _scale;
    static double fl_x_reciprocal = 1.f/ _cameraMatrix->at<double>(0,0);
    static double fl_y_reciprocal = 1.f/ _cameraMatrix->at<double>(1,1);
    for (int y=0; y < fground->rows; ++y) {
        for (int x=0; x < fground->cols; ++x) {
            // Avoing Z NaN values
            if (fground->at<float>(y,x) != fground->at<float>(y,x)) {
                continue;
            }

            double fValX = x/(_scale*1.f);
            double fValY = y/(_scale*1.f);

            fValX = fValX - maxSizeX/2;
            fValY = fValY - maxSizeY/2;

            //Projecting back to image 2D coordinates
            int iValX = (fValX/(fground->at<float>(y,x)*fl_x_reciprocal))+((depth->cols/2)-1);
            int iValY = (fValY/(fground->at<float>(y,x)*fl_y_reciprocal))+((depth->rows/2)-1);

            // Saving found pixel
            if ( iValX >= 0 && iValX < new_mask->cols
                 && iValY >= 0 && iValY < new_mask->rows )
                new_mask->at<unsigned char>(iValY, iValX) = 255;
            else
                BOOST_LOG_TRIVIAL(debug) << "ERR " << iValX << "/" << iValY;
        }
    }
}

void Mask::amplitudeMasking() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    cv::Mat amp_mask;
    amp_mask = *ampl <= 150;
    bitwise_xor(*new_mask,amp_mask,*new_mask);
}

void Mask::morfExMask(cv::Mat &mask) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    int morph_size = 0;
    cv::Mat structuringElement = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size( 2*morph_size + 1, 2*morph_size+1 ),
                cv::Point( morph_size, morph_size ));

    cv::morphologyEx( mask, mask,
                      cv::MORPH_OPEN, structuringElement);

    cv::Mat antiMask = ~mask.clone();
    if (dbg) {
        cv::namedWindow("morphologyEx antiMask", cv::WINDOW_NORMAL);
        cv::imshow("morphologyEx antiMask", antiMask);
    }

    morph_size = 3;
    structuringElement = cv::getStructuringElement(
                cv::MORPH_ELLIPSE, cv::Size( 2*morph_size + 1, 2*morph_size+1 ),
                cv::Point( morph_size, morph_size ));
    cv::morphologyEx( antiMask, antiMask,
                      cv::MORPH_OPEN, structuringElement );

    mask = ~antiMask.clone();

    if (dbg) {
        cv::namedWindow("morphologyEx mask", cv::WINDOW_NORMAL);
        cv::imshow("morphologyEx mask", mask);
    }
    //imshow("after new_mask", mask);

}

#if OCV_VERSION_MAJOR >= 3
#  define LINE_AA cv::LINE_AA
#  define FILLED cv::FILLED
#  define LINE_8 8
#else
#  define LINE_AA CV_AA
#  define FILLED -1
#  define LINE_8 8
#endif


void Mask::fillMaskGaps() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(new_mask->clone(), contours, hierarchy,
                     cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE/*, Point(1,1)*/);

    cv::Mat drawing2 = depth->clone();
    drawing2.convertTo(drawing2, cv::COLOR_GRAY2BGR);
    //cout << "contours.size() " << contours.size() << endl;
    for( size_t i = 0; i < contours.size(); i++ ) {
        cv::drawContours( drawing2, contours, i, cv::Scalar(255, 0, 255), 1,
                          LINE_AA, hierarchy, 0, cv::Point() );
    }
    if (dbg) cv::imshow("drawing2", drawing2);

    double maxArea = -1;
    int maxAreaIndex = -1;
    //cout << "contours.size() " << contours.size() << endl;
    for( size_t i = 0; i < contours.size(); i++ ) {
        double area = contourArea(contours[i]);
        ///cout << "area " << area << endl;
        if (area > 15000) {
            if (contours.size() == 1)
                contours.clear();
            else {
                contours.erase(contours.begin()+i);
                hierarchy.erase(hierarchy.begin()+i);
                i--;
            }
            continue;
        }
        if (area > maxArea) {
            maxArea = area;
            maxAreaIndex = i;
        }
    }

    //cout << "contours.size() " << contours.size() << endl;
    //cout << "maxAreaIndex " << maxAreaIndex << endl;
    if (maxAreaIndex >= 0) {
        if (contours.size() == 1)
            contours.clear();
        else {
            contours.erase(contours.begin()+maxAreaIndex);
            hierarchy.erase(hierarchy.begin()+maxAreaIndex);
        }
    }

    cv::Mat drawing = cv::Mat::zeros( new_mask->size(), CV_8U );
    //cout << "contours.size() " << contours.size() << endl;
    for( size_t i = 0; i < contours.size(); i++ ) {
        double area = contourArea(contours[i]);
        if (area*2 >= maxArea)
            continue;
        cv::drawContours( drawing, contours, i, cv::Scalar(255, 255, 255),
                          FILLED, LINE_8, hierarchy, 0, cv::Point() );
    }
    if (dbg) cv::imshow("drawing", drawing);
    *new_mask |= drawing;
}

void Mask::generate3D() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    if (!img3d) {
        img3d.reset(new cv::Mat(depth->size(),CV_32FC3));
    }

    //Allocating the 3D points

    //Saving parameter from the camera matrix
    static double fl_x_reciprocal = 1.f/_cameraMatrix->at<double>(0,0);
    static double fl_y_reciprocal = 1.f/_cameraMatrix->at<double>(1,1);
    static double center_x = _cameraMatrix->at<double>(0,2);
    static double center_y = _cameraMatrix->at<double>(1,2);

    // Calculates the 3D point for each depth value
    for (int y=0; y < static_cast<int> (depth->rows); ++y) {
        for (int x=0; x < static_cast<int> (depth->cols); ++x) {
            //PointWithRange& point = getPointNoCheck (x, y);
            float *dptr = img3d->ptr<float>(y,x);
            float depthValue = depth->at<float>(y,x);

            dptr[0] = numeric_limits<float>::quiet_NaN (); //X
            dptr[1] = numeric_limits<float>::quiet_NaN (); //Y
            dptr[2] = numeric_limits<float>::quiet_NaN (); //Y
            //cout << " Filtered (" << x << "," << y << ")" << endl;

            //cout << "(" << x << "," << y << ")" << endl;
            //cout << frame.amp().at<unsigned short>(y,x) << endl;
            //point.z = depth;
            //point.x = (static_cast<float> (x) - center_x) * point.z * fl_x_reciprocal;
            //point.y = (static_cast<float> (y) - center_y) * point.z * fl_y_reciprocal;
            /*
         * cout << "x: " <<  (static_cast<float> (x) - center_x) * depthValue * fl_x_reciprocal;
         * cout << ", y: " <<  (static_cast<float> (y) - center_y) * depthValue * fl_y_reciprocal;
         * cout << ", z: " <<  depthValue << endl;
         */
            //Set 3D points
            dptr[0] = (static_cast<float> (x) - center_x) * depthValue * fl_x_reciprocal; //X
            dptr[1] = (static_cast<float> (y) - center_y) * depthValue * fl_y_reciprocal; //Y
            dptr[2] = depthValue; //Z
        }
    }
}
