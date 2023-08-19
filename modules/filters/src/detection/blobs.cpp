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
#include <iomanip>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <toffy/filter_helpers.hpp>
#include <toffy/btaFrame.hpp>
#include <toffy/common/pointTransfom.hpp>

#include <toffy/detection/detectedObject.hpp>
#include <toffy/detection/blobs.hpp>



#ifdef toffy_DEBUG
static const bool dbgShape=true;
#else
static const bool dbgShape=false;
#endif


using namespace std;
using namespace cv;
using namespace toffy;
using namespace toffy::detection;

std::size_t Blobs::_filter_counter = 1;
const std::string Blobs::id_name = "blobs";

Blobs::Blobs(): Filter(Blobs::id_name,_filter_counter),
    in_img("fg"), in_ampl("ampl"), out_blobs("blobs"),
    _minSize(40), _morphoSize(1), _morphoIter(1), _morphoType(0),
    refineBlobs(false), _morpho(false), sharpenEdges(false),
    _filterInternals(true)
{
    _filter_counter++;
}

Blobs::~Blobs() {}


void Blobs::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    in_img = pt.get<string>("inputs.img",in_img);
    out_blobs = pt.get<string>("outputs.blobs",out_blobs);

    _minSize = pt.get<int>("options.minSize", _minSize);

    //amplFilter = pt.get<bool>("options.amplFilter", amplFilter);
    //minAmpl = pt.get<int>("options.minAmpl", minAmpl);

    refineBlobs = pt.get<bool>("options.refineBlobs", refineBlobs);
    _morpho = pt.get<bool>("options.morpho", _morpho);
    _morphoSize = pt.get<int>("options.morphoSize", _morphoSize);
    _morphoIter = pt.get<int>("options.morphoIter", _morphoIter);
    _morphoType = pt.get<int>("options.morphoType", _morphoType);

    sharpenEdges = pt.get<bool>("options.sharpenEdges", sharpenEdges);
    _filterInternals = pt.get<bool>("options.filterInternals", _filterInternals);
}

boost::property_tree::ptree Blobs::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("inputs.img", in_img);
    pt.put("outputs.blobs", out_blobs);

    pt.put("options.minSize", _minSize);

    //pt.put("options.amplFilter", amplFilter);
    //pt.put("options.minAmpl", minAmpl);

    pt.put("options.refineBlobs", refineBlobs);
    pt.put("options.morpho", _morpho);
    pt.put("options.morphoSize", _morphoSize);
    pt.put("options.morphoIter", _morphoIter);
    pt.put("options.morphoType", _morphoType);

    pt.put("options.sharpenEdges", sharpenEdges);
    pt.put("options.filterInternals", _filterInternals);

    return pt;
}

bool Blobs::filter(const toffy::Frame& in, toffy::Frame& out)
{
    unsigned int fc;
    try {
        fc = in.getUInt(btaFc);
        BOOST_LOG_TRIVIAL(debug) << id() << ": Found input fc: " << btaFc;
    } catch(const boost::bad_any_cast &) {
        BOOST_LOG_TRIVIAL(warning) <<
                                      "Could not cast input " << btaFc;
        return false;
    }
    boost::shared_ptr<cv::Mat> inImg;
    try {
        inImg = in.getMatPtr(in_img);
        BOOST_LOG_TRIVIAL(debug) << id() << ": Found input in_img: " << in_img;
    } catch(const boost::bad_any_cast &) {
        BOOST_LOG_TRIVIAL(warning) <<
                                      "Could not cast input " << in_img <<
                                      ", filter  " << id() <<" not applied.";
        return true;
    }

    boost::shared_ptr<cv::Mat> ampl;
    try {
        ampl = in.getMatPtr(in_ampl);
        BOOST_LOG_TRIVIAL(debug) << id() << ": Found input in_ampl: " << in_ampl;
    } catch(const boost::bad_any_cast &) {
        BOOST_LOG_TRIVIAL(warning) <<
                                      "Could not cast input " << in_ampl <<
                                      ", filter  " << id() <<" not applied.";
        return true;
    }

    boost::shared_ptr<std::vector<DetectedObject*> > blobs;
    try {
        blobs = boost::any_cast<boost::shared_ptr<std::vector<DetectedObject*> > >(out.getData(out_blobs));
        blobs->clear();
        BOOST_LOG_TRIVIAL(debug) << id() << ": Found output out_blobs: " << out_blobs;
    } catch(const boost::bad_any_cast &) {
        BOOST_LOG_TRIVIAL(warning) << "Could not find object vector.";
        blobs.reset(new std::vector<DetectedObject*>);
        //return true;
    }
    blobs->clear();

    findBlobs(*inImg, *ampl, fc, *blobs);

    out.addData(out_blobs, blobs);

    cout << id() << ": blobs saved into " << out_blobs << " size  =" << blobs->size() << endl;
    return true;
}


void Blobs::findBlobs(cv::Mat& img, cv::Mat& ampl, int fc, std::vector<DetectedObject*>& detObj)
{
    UNUSED(ampl);
    BOOST_LOG_TRIVIAL(debug) << __FILE__ << ": " <<__FUNCTION__;

    //Filter
    Mat m = img.clone();
    //m.setTo(0, m > 0.01);


    if (dbg) imshow("m " , m);
    //cvv::showImage(m, CVVISUAL_LOCATION, "m");

    // Convert to 8bit
    double max, min;
    cv::minMaxLoc(m, &min, &max);
    m.convertTo(m,CV_8U,255.0/(max-min),-255.0*min/(max-min));

    if (dbg) imshow("m2 " , m);
    //cvv::showImage(m, CVVISUAL_LOCATION, "m2");

    //Apply the optional morphology operation
    if (_morpho) {
        Mat element = getStructuringElement( MORPH_RECT, Size( 2*_morphoSize + 1, 2*_morphoSize+1 ), Point( _morphoSize, _morphoSize ) );
        morphologyEx( m, m, _morphoType, element, Point(-1,-1),_morphoIter);

        /*Mat edges;
    Canny( m, edges, 50, 50*2, 3 );
    int morph_size = 0;
    Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    morphologyEx( edges, edges, MORPH_CLOSE, element, Point(-1,-1),3 );
    //dilate(edges,edges);
    //cv::copyMakeBorder(edges, edges, 1, 1, 1, 1, cv::BORDER_CONSTANT,Scalar(1));
    //if (dbg) imshow("edges " , edges);
    m.setTo(0,edges);
    if (dbg) imshow("edges " , edges);
    //cvv::showImage(edges, CVVISUAL_LOCATION, "edges");*/
    }


    //cvv::showImage(m, CVVISUAL_LOCATION, "blobs to track");

    if (dbg) imshow("blobs to track " , m);

    //Find the blobs contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(m, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

    cout << "contours: " << contours.size() << endl;

    //Shows all found contours
    if (dbg) {
        Mat imgCopy(img.size(),CV_8UC3, Scalar(0,0,0));
        RNG rng(12345);
        for (size_t i = 0; i < contours.size(); i++) {
            if ( hierarchy[i][3] == -1 ) {
                Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
                drawContours( imgCopy, contours, i, color, -1, LINE_AA);
            }
        }
        imshow("detBlobs", imgCopy);
        //cvv::showImage(imgCopy, CVVISUAL_LOCATION, "detBlobs");
    }

    /*
    for( size_t i = 0; i< contours.size(); i++ )
    vector<Point> contour = contours[i];
    for( size_t j = 1; j< contours.size(); j++ ) {
        vector<Point> contour2 = contours[j];
        for( size_t k = 1; k< contour2.size(); k++ ) {

        }
    }
    {*/

    DetectedObject *obj;
    Moments mu;
    Point2f mc;

    if (dbg) cout << "Blobs: # = " << contours.size() << endl;

    for( size_t i = 0; i< contours.size(); i++ ) {
        mu = moments( contours[i] );
        mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

        if ( mc.x != mc.x ) { // skip nan value areas
            //cout << "SKIP nan " << i << " " << mc << " " <<  endl;
            continue;
        }

        //Filter by size
        if ( mu.m00 < _minSize) {
            continue;
        }
        //Filter if has internal holes
        if (_filterInternals && hierarchy[i][3] != -1 ) {
            continue;
        }

        //Create and object for each blob
        obj = new DetectedObject();
        obj->fc = fc;

        obj->massCenter = mc;

        obj-> mo = mu;

        //Moving the massCenter if it is outside or in a hole
        if (pointPolygonTest(contours[i], obj->massCenter, false) < 0 || img.at<float>(obj->massCenter.y,obj->massCenter.x) < .05) {
            obj->massCenter = contours[i][0];
        }

        if (refineBlobs) {
            /*if (amplFilter) {
                Mat im;
                img.copyTo(im, m);
                refineBlob(im, i, obj);
            } else*/
            refineBlob(img, i, obj);

        } else {
            obj->idx = i;
            obj->contour = contours[i];
            obj->contours.reset(new vector<vector<Point> >(contours));
            obj->hierarchy.reset(new vector<Vec4i> (hierarchy));
            obj->mo = mu;
            obj->size = contourArea(contours[i]);
        }

        obj->massCenterZ = img.at<float>(obj->massCenter);
        obj->massCenter3D = commons::pointTo3D(obj->massCenter, obj->massCenterZ);

        //adding detected object to the output list
        if(obj->idx >= 0 )
            detObj.push_back(obj);

        if (dbgShape) {
            DetectedObject *o = obj;
            cout << setprecision(6);
            cout << "BLB " << o->idx << "\t"
                 << o->massCenter.x << "\t"<< o->massCenter.y << "\t"
                    /*
                  << o->mo.m00 << "\t" << o->mo.m10 << "\t"
                  << o->mo.m01 << "\t" << o->mo.m20 << "\t"
                  << o->mo.m11 << "\t" << o->mo.m02 << "\t"
                  << o->mo.m30 << "\t" << o->mo.m21 << "\t"
                  << o->mo.m12 << "\t" << o->mo.m03 << "\t"
                */
                 << "mu\t"
                 << o->mo.mu20 << "\t" << o->mo.mu11 << "\t"
                 << o->mo.mu02 << "\t" << o->mo.mu30 << "\t"
                 << o->mo.mu21 << "\t" << o->mo.mu12 << "\t"
                 << o->mo.mu03 << "\t"
                 << "nu\t"
                 << o->mo.nu20 << "\t" << o->mo.nu11 << "\t"
                 << o->mo.nu02 << "\t" << o->mo.nu30 << "\t"
                 << o->mo.nu21 << "\t" << o->mo.nu12 << "\t"
                 << o->mo.nu03

                 << endl;
        }
    }

    // shows blobs after filtering
    if (dbg) {
        Mat imgCopy(img.size(),CV_8UC3, Scalar(0,0,0));
        //img.convertTo(imgCopy,CV_GRAY2BGR);
        RNG rng(12345);
        for (size_t i = 0; i < detObj.size(); i++) {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( imgCopy, *detObj[i]->contours, detObj[i]->idx, color,
                          FILLED, LINE_AA, *detObj[i]->hierarchy);
            circle( imgCopy, detObj[i]->massCenter, 2, Scalar( 0, 255, 255 ), FILLED, LINE_AA);
        }
        imshow("detttttBlobs", imgCopy);
        //cvv::showImage(imgCopy, CVVISUAL_LOCATION, "detttttBlobs");
    }
}

void Blobs::refineBlob(cv::Mat& dist, int fc,
                       toffy::detection::DetectedObject* o)
{
    UNUSED(fc);
    Mat mask(dist.size(), CV_8UC1);

    float dlt=0.025;       // 2.5cm max depth change between pixels

    if (dbg) imshow("dist", dist );
    //cvv::showImage(dist, CVVISUAL_LOCATION, "dist");

    mask.setTo(0);

    cv::copyMakeBorder(mask, mask, 1, 1, 1, 1, cv::BORDER_REPLICATE);

    Mat copyMask = mask.clone();

    floodFill(dist, mask, o->massCenter+Point2f(1.1), Scalar(255), 0, Scalar(dlt), Scalar(dlt), 4 | FLOODFILL_MASK_ONLY | ( 255<<8) );

    if (dbg) imshow("FLOOD", mask );
    //cvv::showImage(mask, CVVISUAL_LOCATION, "FLOOD");

    mask.setTo(0,copyMask);
    Mat m(mask, Rect(1,1,mask.cols-2, mask.rows-2));

    if (dbg) imshow("fmask2", mask );
    if (dbg) imshow("FLOOD2", m );
    //cvv::showImage(mask, CVVISUAL_LOCATION, "fmask2");
    //cvv::showImage(m, CVVISUAL_LOCATION, "FLOOD2");

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(m, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

    if (contours.size() > 0) {
        o->idx = 0;
        //Get the right id when small objects are detected in redefine.
        //Looking for the biggest object.
        unsigned int max_size = (int)contours[0].size();
        for( size_t i = 1; i< contours.size(); i++ ) {
            if(max_size < contours[i].size()) {
                max_size = contours[i].size();
                o->idx = i;
            }
        }
        o->contours.reset(new vector<vector<Point> >(contours));
        o->hierarchy.reset(new vector<Vec4i> (hierarchy));

        o->contour = contours[o->idx];

        /*if (dbg) {
            Mat imgCopy(dist.size(),CV_8UC3, Scalar(0,0,0));
            //img.convertTo(imgCopy,CV_GRAY2BGR);
            RNG rng(12345);
            for (size_t i = 0; i < contours.size(); i++) {
            Scalar color = Scalar( rng.uniform(0, 255),
                                    rng.uniform(0,255),
                                    rng.uniform(0,255) );
            drawContours( imgCopy, contours, i, color,
                    FILLED, LINE_AA, hierarchy);
            circle( imgCopy, o->massCenter, 2, Scalar( 0, 255, 255 ),
                    FILLED, LINE_AA);
            }
            imshow("<<<<<", imgCopy);
        }*/

        o->mo = moments( contours[o->idx] );
        o->massCenter = Point2f( o->mo.m10/o->mo.m00 , o->mo.m01/o->mo.m00 );

        /*if (dbg) {
        Mat imgCopy(dist.size(),CV_8UC3, Scalar(0,0,0));
        //img.convertTo(imgCopy,CV_GRAY2BGR);
        RNG rng(12345);
        for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( imgCopy, contours, i, color,
                  FILLED, LINE_AA, hierarchy);
        circle( imgCopy, o->massCenter, 2, Scalar( 0, 255, 255 ), FILLED, LINE_AA);
        }
        imshow("qqqqq", imgCopy);
    }*/
    } else {
        o->idx = -1;
    }

}
