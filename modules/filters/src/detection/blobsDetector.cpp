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
#include <iomanip>
#include <iostream>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <toffy/btaFrame.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <toffy/detection/detectedObject.hpp>

#include "toffy/detection/blobsDetector.hpp"

static const bool dbg = false;
static const bool dbgShape = false;

using namespace std;
using namespace cv;
using namespace toffy;
using namespace toffy::detection;

std::size_t BlobsDetector::_filter_counter = 1;
const std::string BlobsDetector::id_name = "blobsDetector";

BlobsDetector::BlobsDetector()
    : Filter(BlobsDetector::id_name),
      in_img("fg"),
      in_ampl("ampl"),
      out_blobs("blobs"),
      _minSize(40),
      amplFilter(false),
      minAmpl(220),
      refineBlobs(false),
      morpho(false),
      sharpenEdges(false) {
  _filter_counter++;
}

BlobsDetector::~BlobsDetector() {}

void BlobsDetector::updateConfig(const boost::property_tree::ptree& pt) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

  using namespace boost::property_tree;

  Filter::updateConfig(pt);

  in_img = pt.get<string>("inputs.img", in_img);
  out_blobs = pt.get<string>("outputs.blobs", out_blobs);

  _minSize = pt.get<int>("options.minSize", _minSize);

  amplFilter = pt.get<bool>("options.amplFilter", amplFilter);
  minAmpl = pt.get<int>("options.minAmpl", minAmpl);

  refineBlobs = pt.get<bool>("options.refineBlobs", refineBlobs);
  morpho = pt.get<bool>("options.morpho", morpho);

  sharpenEdges = pt.get<bool>("options.sharpenEdges", sharpenEdges);

  _params.thresholdStep = pt.get<float>("options.thresholdStep", 10);
  _params.minThreshold = pt.get<float>("options.minThreshold", 10);
  _params.maxThreshold = pt.get<float>("options.maxThreshold", 220);
  _params.minRepeatability = pt.get<int>("options.minRepeatability", 2);
  _params.minDistBetweenBlobs =
      pt.get<float>("options.minDistBetweenBlobs", 10);

  _params.filterByColor = pt.get<bool>("options.filterByColor", false);
  _params.blobColor = pt.get<float>("options.blobColor", 0);

  _params.filterByArea = pt.get<bool>("options.filterByArea", false);
  _params.minArea = pt.get<float>("options.minArea", 25);
  _params.maxArea = pt.get<float>("options.maxArea", 5000);

  _params.filterByCircularity =
      pt.get<bool>("options.filterByCircularity", false);
  _params.minCircularity = pt.get<float>("options.minCircularity", 0.9f);
  _params.maxCircularity = pt.get<float>("options.maxCircularity", (float)1e37);

  _params.filterByInertia = pt.get<bool>("options.filterByInertia", false);
  _params.minInertiaRatio = pt.get<float>("options.minInertiaRatio", 0.1f);
  _params.maxInertiaRatio =
      pt.get<float>("options.maxInertiaRatio", (float)1e37);

  _params.filterByConvexity = pt.get<bool>("options.filterByConvexity", false);
  _params.minConvexity = pt.get<float>("options.minConvexity", 0.95f);
  _params.maxConvexity = pt.get<float>("options.maxConvexity", (float)1e37);
}

boost::property_tree::ptree BlobsDetector::getConfig() const {
  boost::property_tree::ptree pt;

  pt = Filter::getConfig();

  pt.put("inputs.img", in_img);
  pt.put("outputs.blobs", out_blobs);

  pt.put("options.minSize", _minSize);

  pt.put("options.amplFilter", amplFilter);
  pt.put("options.minAmpl", minAmpl);

  pt.put("options.refineBlobs", refineBlobs);
  pt.put("options.morpho", morpho);

  pt.put("options.sharpenEdges", sharpenEdges);

  pt.put("options.thresholdStep", _params.thresholdStep);
  pt.put("options.minThreshold", _params.minThreshold);
  pt.put("options.maxThreshold", _params.maxThreshold);
  pt.put("options.minRepeatability", _params.minRepeatability);
  pt.put("options.minDistBetweenBlobs", _params.minDistBetweenBlobs);

  pt.put("options.filterByColor", _params.filterByColor);
  pt.put("options.blobColor", _params.blobColor);

  pt.put("options.filterByArea", _params.filterByArea);
  pt.put("options.minArea", _params.minArea);
  pt.put("options.maxArea", _params.maxArea);

  pt.put("options.filterByCircularity", _params.filterByCircularity);
  pt.put("options.minCircularity", _params.minCircularity);
  pt.put("options.maxCircularity", _params.maxCircularity);

  pt.put("options.filterByInertia", _params.filterByInertia);
  pt.put("options.minInertiaRatio", _params.minInertiaRatio);
  pt.put("options.maxInertiaRatio", _params.maxInertiaRatio);

  pt.put("options.filterByConvexity", _params.filterByConvexity);
  pt.put("options.minConvexity", _params.minConvexity);
  pt.put("options.maxConvexity", _params.maxConvexity);

  return pt;
}

bool BlobsDetector::filter(const toffy::Frame& in, toffy::Frame& out) {
  matPtr inImg;
  matPtr ampl;
  DetObjectsPtr blobs;
  // unsigned int fc;

  try {
    fc = in.getUInt(btaFc);
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning) << "Could not cast input " << btaFc;
    return false;
  }

  try {
    ts = in.getUInt("ts");
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning) << "Could not cast input "
                               << "ts";
    return false;
  }

  try {
    inImg = in.getMatPtr(in_img);
    ampl = in.getMatPtr(in_ampl);
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning) << "Could not cast input " << in_img
                               << ", filter  " << id() << " not applied.";
    return false;
  }

  try {
    blobs = boost::any_cast<DetObjectsPtr >(
        out.getData(out_blobs));
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning) << "Could not find object vector. Creating one";
    blobs.reset(new std::vector<DetectedObject*>);
  }

  findBlobs(*inImg, *ampl, fc, *blobs);

  // out.addData(out_blobs, blobs);

  // cout << id() << ": blobs saved into " << out_blobs << " size  =" <<
  // blobs->size() << endl;
  return true;
}

void BlobsDetector::findBlobs(cv::Mat& img, cv::Mat& ampl, int fc,
                              std::vector<DetectedObject*>& detObj) {
  BOOST_LOG_TRIVIAL(debug) << __FILE__ << ": " << __FUNCTION__;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  detObj.clear();
  // Rect rect(0,0,img.cols,img.rows);
  // Mat roi2(img,rect);

  Mat m = img > 0.01;

  if (amplFilter) {
    m &= (ampl > minAmpl);
  }

  img.copyTo(m, m);

  // medianBlur(m, m, 3);
  // int morph_size = 1;
  // Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1,
  // 2*morph_size+1 ), Point( morph_size, morph_size ) );
  // morphologyEx( m, m, MORPH_OPEN, element );

  if (dbg) imshow("m ", m);

  // Convert to 8bit
  double max, min;
  cv::minMaxLoc(m, &min, &max);
  m.convertTo(m, CV_8U, 255.0 / (max - min), -255.0 * min / (max - min));

  // medianBlur(m, m, 3);
  // morphologyEx( m, m, operation, element );
  if (dbg) imshow("m2 ", m);

  if (morpho) {
    Mat edges;
    Canny(m, edges, 50, 50 * 2, 3);
    int morph_size = 0;
    Mat element = getStructuringElement(
        MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1),
        Point(morph_size, morph_size));
    morphologyEx(edges, edges, MORPH_CLOSE, element, Point(-1, -1), 3);
    // dilate(edges,edges);
    // cv::copyMakeBorder(edges, edges, 1, 1, 1, 1,
    // cv::BORDER_CONSTANT,Scalar(1)); if (dbg) imshow("edges " , edges);
    m.setTo(0, edges);
    if (dbg) imshow("edges ", edges);
  }

  if (dbg) imshow("blobs to track ", m);
  vector<KeyPoint> keyImg;
#if OCV_VERSION_MAJOR >= 3
  Ptr<cv::Feature2D> b;

  b = SimpleBlobDetector::create(_params);

  Ptr<SimpleBlobDetector> sbd = b.dynamicCast<SimpleBlobDetector>();

  sbd->detect(m, keyImg, Mat());
#else
  SimpleBlobDetector sbd;
  sbd.detect(m, keyImg, Mat());
#endif
  cout << "keyPoints found: " << keyImg.size() << endl;
  Mat result(m.size(), CV_8UC3);
  drawKeypoints(
      m, keyImg, result, Scalar::all(-1),
      DrawMatchesFlags::DEFAULT & DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  imshow("rest", result);
  // waitKey();

  /*
  findContours(m, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

  if (dbg) {
      Mat imgCopy(img.size(),CV_8UC3, Scalar(0,0,0));
      RNG rng(12345);
      for (size_t i = 0; i < contours.size(); i++) {
          if ( hierarchy[i][3] == -1 ) {
              Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255),
  rng.uniform(0,255) ); drawContours( imgCopy, contours, i, color, -1, LINE_AA);
          }
      }
      imshow("detBlobs", imgCopy);
  }

  DetectedObject *obj;
  Moments mu;
  Point2f mc;

  if (dbg) cout << "Blobs: # = " << contours.size() << endl;

  for( size_t i = 0; i< contours.size(); i++ )
      {
          mu = moments( contours[i] );
          mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

          //cout << "CHK " << mc << endl;
          if ( mc.x != mc.x ) { // skip nan value areas
              //cout << "SKIP nan " << i << " " << mc << " " <<  endl;
              continue;
          }
          if ( mu.m00 < _minSize) { // skip small areas
              continue;
          }

          if ( hierarchy[i][3] != -1 ) {
              continue;
          }

          obj = new DetectedObject();
          obj->fc = fc;

          obj->massCenter = mc;
          obj-> mo = mu;

          //Moving the massCenter if it is outside or in a hole
          if (pointPolygonTest(contours[i], obj->massCenter, false) < 0 ||
  img.at<float>(obj->massCenter.y,obj->massCenter.x) < .05) { obj->massCenter =
  contours[i][0];
          }

          if (refineBlobs) {
              if (amplFilter) {
                  Mat im;
                  img.copyTo(im, m);
                  refineBlob(im, i, obj);
              } else
                  refineBlob(img, i, obj);
              if(obj->idx >= 0 )
                  detObj.push_back(obj);
          } else {
              obj->idx = i;
              obj->contour = contours[i];
              obj->contours.reset(new vector<vector<Point> >(contours));
              obj->hierarchy.reset(new vector<Vec4i> (hierarchy));
              obj->mo = mu;

              detObj.push_back(obj);
          }


          if (dbgShape) {
              DetectedObject *o = obj;
              cout << setprecision(6);
              cout << "BLB " << o->idx << "\t"
                   << o->massCenter.x << "\t"<< o->massCenter.y << "\t"
                  / *
                    << o->mo.m00 << "\t" << o->mo.m10 << "\t"
                    << o->mo.m01 << "\t" << o->mo.m20 << "\t"
                    << o->mo.m11 << "\t" << o->mo.m02 << "\t"
                    << o->mo.m30 << "\t" << o->mo.m21 << "\t"
                    << o->mo.m12 << "\t" << o->mo.m03 << "\t"
                  * /
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

  if (dbg) {
      Mat imgCopy(img.size(),CV_8UC3, Scalar(0,0,0));
      //img.convertTo(imgCopy,CV_GRAY2BGR);
      RNG rng(12345);
      for (size_t i = 0; i < detObj.size(); i++) {
          Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255),
  rng.uniform(0,255) ); drawContours( imgCopy, *detObj[i]->contours,
  detObj[i]->idx, color, FILLED, LINE_AA, *detObj[i]->hierarchy); circle(
  imgCopy, detObj[i]->massCenter, 2, Scalar( 0, 255, 255 ), FILLED, LINE_AA);
      }
      imshow("detttttBlobs", imgCopy);
  }
*/
}

void BlobsDetector::refineBlob(cv::Mat& dist, int fc,
                               toffy::detection::DetectedObject* o) {
  Mat mask(dist.size(), CV_8UC1);

  float dlt = 0.025;  // 2.5cm max depth change between pixels

  if (dbg) imshow("dist", dist);

  mask.setTo(0);

  cv::copyMakeBorder(mask, mask, 1, 1, 1, 1, cv::BORDER_REPLICATE);

  Mat copyMask = mask.clone();

  floodFill(dist, mask, o->massCenter + Point2f(1.1), Scalar(255), 0,
            Scalar(dlt), Scalar(dlt), 4 | FLOODFILL_MASK_ONLY | (255 << 8));

  if (dbg) imshow("FLOOD", mask);

  mask.setTo(0, copyMask);
  Mat m(mask, Rect(1, 1, mask.cols - 2, mask.rows - 2));

  if (dbg) imshow("fmask2", mask);
  if (dbg) imshow("FLOOD2", m);

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(m, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

  if (contours.size() > 0) {
    o->idx = 0;
    // Get the right id when small objects are detected in redefine.
    // Looking for the biggest object.
    unsigned int max_size = (int)contours[0].size();
    for (size_t i = 1; i < contours.size(); i++) {
      if (max_size < contours[i].size()) {
        max_size = contours[i].size();
        o->idx = i;
      }
    }

    o->contours.reset(new vector<vector<Point> >(contours));
    o->hierarchy.reset(new vector<Vec4i>(hierarchy));

    o->contour = contours[o->idx];

    /*if (dbg) {
        Mat imgCopy(dist.size(),CV_8UC3, Scalar(0,0,0));
        //img.convertTo(imgCopy,CV_GRAY2BGR);
        RNG rng(12345);
        for (size_t i = 0; i < contours.size(); i++) {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255),
    rng.uniform(0,255) ); drawContours( imgCopy, contours, i, color, FILLED,
    LINE_AA, hierarchy); circle( imgCopy, o->massCenter, 2, Scalar( 0, 255, 255
    ), FILLED, LINE_AA);
        }
        imshow("<<<<<", imgCopy);
    }*/

    o->mo = moments(contours[o->idx]);
    o->massCenter = Point2f(o->mo.m10 / o->mo.m00, o->mo.m01 / o->mo.m00);

    o->fc = fc;
    o->cts = ts;
    o->ts = boost::posix_time::microsec_clock::local_time();

    /*if (dbg) {
        Mat imgCopy(dist.size(),CV_8UC3, Scalar(0,0,0));
        //img.convertTo(imgCopy,CV_GRAY2BGR);
        RNG rng(12345);
        for (size_t i = 0; i < contours.size(); i++) {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255),
    rng.uniform(0,255) ); drawContours( imgCopy, contours, i, color, FILLED,
    LINE_AA, hierarchy); circle( imgCopy, o->massCenter, 2, Scalar( 0, 255, 255
    ), FILLED, LINE_AA);
        }
        imshow("qqqqq", imgCopy);
    }*/
  } else {
    o->idx = -1;
  }
}
