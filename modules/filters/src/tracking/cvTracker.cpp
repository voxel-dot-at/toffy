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

#include <boost/log/trivial.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <toffy/tracking/cvTracker.hpp>


using namespace toffy;
using namespace toffy::tracking;
using namespace cv;
using namespace std;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif

std::size_t CVTracker::_filter_counter = 1;
const std::string CVTracker::id_name = "cvTracker";

CVTracker::CVTracker(): Filter(CVTracker::id_name, _filter_counter),
    _in_vec("dect"), _in_fc("fc"), _in_img("img"),
    _out_img(_in_img), _out_objects("objects"),
    _out_count("count"), maxMergeDistance(20.),
    _render_image(false)
{
    _filter_counter++;
    //tracker = cv::Tracker::create( "MIL" );

}

void CVTracker::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _in_vec = pt.get<string>("inputs.vec",_in_vec);
    _in_fc = pt.get<string>("inputs.fc",_in_fc);
    _in_img = pt.get<string>("inputs.img",_in_img);

    _out_img = pt.get<string>("outputs.img",_out_img);
    _out_objects = pt.get<string>("outputs.objects",_out_objects);
    _out_count = pt.get<string>("outputs.count",_out_count);

    maxMergeDistance = pt.get<double>("options.maxMergeDistance",maxMergeDistance);
    _render_image = pt.get<bool>("options.renderImage",_render_image);

}

boost::property_tree::ptree CVTracker::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();
    //pt.put("options.max_x", _max_x);

    pt.put("inputs.vec", _in_vec);
    pt.put("inputs.fc", _in_fc);
    pt.put("inputs.img", _in_img);
    pt.put("outputs.img", _out_img);
    pt.put("outputs.objects", _out_objects);
    pt.put("outputs.count", _out_count);

    pt.put("options.renderImage", _render_image);
    pt.put("options.maxMergeDistance", maxMergeDistance);
    return pt;
}

bool CVTracker::filter(const Frame& in, Frame& out) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    toffy::Filter::setLoggingLvl();

    //List of detected objects
    BOOST_LOG_TRIVIAL(debug) << "Getting _in_vec: " << _in_vec;
    boost::shared_ptr<std::vector<detection::DetectedObject*> > detObj;
    try {
        detObj = boost::any_cast<boost::shared_ptr<std::vector<detection::DetectedObject*> > >(in.getData(_in_vec));
    } catch(const boost::bad_any_cast &) {
        BOOST_LOG_TRIVIAL(warning) << "Could not find object vector: " <<
                                      _in_vec << ". " << id();
        return true;
    }

    BOOST_LOG_TRIVIAL(debug) << "Got _in_vec(" << _in_vec << "): " << detObj->size();

    // TODO Where come img from?, why we mask it with the objects.
    BOOST_LOG_TRIVIAL(debug) << "Getting got _in_img: " << _in_img;
    boost::shared_ptr<cv::Mat> img;
    try {
        img = in.getMatPtr(_in_img);
    } catch(const boost::bad_any_cast &) {
        BOOST_LOG_TRIVIAL(warning) <<
                                      "Could not cast input " << _in_img <<
                                      ", filter  " << id() <<" does not show objects.";
        img.reset(new cv::Mat(cv::Size(160,120),CV_8UC3, Scalar(0,0,0)));
        out.addData(_in_img,img);
    }
    img.reset(new cv::Mat(cv::Size(160,120),CV_8UC3, Scalar(0,0,0)));


    for (size_t i = 0; i < detObj->size(); i++) {
        cv::drawContours( *img, *(detObj->at(i)->contours), detObj->at(i)->idx, detObj->at(i)->color,
                      FILLED, LINE_AA, *(detObj->at(i)->hierarchy));
    }

    Mat show = img->clone();
    for (size_t i = 0; i < detObj->size(); i++) {
        RotatedRect rr = cv::minAreaRect(detObj->at(0)->contour);
        cv::rectangle(show, rr.boundingRect(), cv::Scalar( 0, 255, 0 ), 1 );
        Point2f rect_points[4];
        rr.points(rect_points);
        for( int j = 0; j < 4; j++ )
                  line( show, rect_points[j], rect_points[(j+1)%4], cv::Scalar( 255, 0, 0 ), 1, 8 );
        //circle( imgCopy, o->massCenter, 2, Scalar( 0, 255, 255 ), FILLED, LINE_AA);
    }

    cv::namedWindow("Trackeds", cv::WINDOW_NORMAL);
    cv::imshow("Trackeds", show);

    if(!tracker) {
        bbox = cv::minAreaRect(detObj->at(0)->contour).boundingRect();
#if (OCV_VERSION_MAJOR >= 3 ) && (OCV_VERSION_MINOR > 1) || (CV_VERSION_MAJOR >= 4)
        tracker = cv::TrackerKCF::create( );
#else
        tracker = cv::Tracker::create( "KCF" );
#endif
        tracker->init(*img,bbox);
    } else {
        bbox = cv::minAreaRect(detObj->at(0)->contour).boundingRect();
        tracker->update(*img, bbox);
        // Draw bounding box
        cv::rectangle(*img, bbox, cv::Scalar( 255, 0, 0 ), 1 );

        // Display result
        cv::namedWindow("Tracking", cv::WINDOW_NORMAL);
        cv::imshow("Tracking", *img);
    }

    return true;
}

void CVTracker::showObjects(cv::Mat& depth) {
    double max, min;
    minMaxIdx(depth, &min, &max);
    depth.convertTo(depth,CV_8U,255.0/(max-min),-255.0*min/(max-min));
    cvtColor(depth,depth, COLOR_GRAY2RGB);

    //BOOST_LOG_TRIVIAL(debug) << "showObjects " << blobs.size();
    for( size_t i = 0; i< blobs.size(); i++ )
    {
        if (blobs[i]->first_fc == _fc) {
            circle(depth, blobs[i]->massCenter, 4, blobs[i]->color);
            continue;
        }
        try {
            if (blobs[i]->fc == _fc) {
                vector<vector<Point> > contours;
                contours.push_back(blobs[i]->contour);
                drawContours( depth, contours, 0, blobs[i]->color, 1, LINE_AA, noArray(), 0, Point() );
                circle(depth, blobs[i]->massCenter, 2, blobs[i]->color);
            }
        } catch (std::exception &e) {
            LOG(warning) << "auweh - could not paint blobs.s=" << blobs.size() << " i= " << i ;
        }

    }

    if (dbg) cv::imshow("objectsDetc", depth);

    return;
}

