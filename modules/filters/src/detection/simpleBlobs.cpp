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
#if CV_VERSION_MAJOR >= 3
#  include <opencv2/highgui.hpp>
#  include <opencv2/imgproc.hpp>
#else
#  include <opencv2/highgui/highgui.hpp>
#  include <opencv2/imgproc/imgproc.hpp>
#endif

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <toffy/btaFrame.hpp>
#include <toffy/common/pointTransfom.hpp>
#include <toffy/detection/detectedObject.hpp>
#include <toffy/detection/simpleBlobs.hpp>

#ifdef toffy_DEBUG
const bool dbg = true;
const bool dbgShape = true;
#else
const bool dbg = false;
const bool dbgShape = false;
#endif

using namespace std;
using namespace cv;
using namespace toffy;
using namespace toffy::detection;

std::size_t SimpleBlobs::_filter_counter = 1;
const std::string SimpleBlobs::id_name = "SimpleBlobs";

SimpleBlobs::SimpleBlobs()
    : Filter(SimpleBlobs::id_name, _filter_counter),
      in_img("fg"),
      in_ampl("ampl"),
      out_blobs("blobs"),
      _minSize(40),
      _morphoSize(1),
      _morphoIter(1),
      _morphoType(0),
      _morpho(false),
      sharpenEdges(false),
      _filterInternals(true) {
  _filter_counter++;
}

SimpleBlobs::~SimpleBlobs() {}

void SimpleBlobs::updateConfig(const boost::property_tree::ptree& pt) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

  using namespace boost::property_tree;

  Filter::updateConfig(pt);

  in_img = pt.get<string>("inputs.img", in_img);    // foreground / binary image
  in_ampl = pt.get<string>("inputs.ampl", in_img);  // amplitudes / gray scale

  out_blobs = pt.get<string>("outputs.blobs", out_blobs);

  _minSize = pt.get<int>("options.minSize", _minSize);

  _morpho = pt.get<bool>("options.morpho", _morpho);
  _morphoSize = pt.get<int>("options.morphoSize", _morphoSize);
  _morphoIter = pt.get<int>("options.morphoIter", _morphoIter);
  _morphoType = pt.get<int>("options.morphoType", _morphoType);

  sharpenEdges = pt.get<bool>("options.sharpenEdges", sharpenEdges);
  _filterInternals = pt.get<bool>("options.filterInternals", _filterInternals);
}

boost::property_tree::ptree SimpleBlobs::getConfig() const {
  boost::property_tree::ptree pt;

  pt = Filter::getConfig();

  pt.put("inputs.img", in_img);
  pt.put("outputs.blobs", out_blobs);

  pt.put("options.minSize", _minSize);

  pt.put("options.morpho", _morpho);
  pt.put("options.morphoSize", _morphoSize);
  pt.put("options.morphoIter", _morphoIter);
  pt.put("options.morphoType", _morphoType);

  pt.put("options.sharpenEdges", sharpenEdges);
  pt.put("options.filterInternals", _filterInternals);

  return pt;
}

bool SimpleBlobs::filter(const toffy::Frame& in, toffy::Frame& out) {
  unsigned int fc;
  try {
    fc = in.getUInt(btaFc);
    BOOST_LOG_TRIVIAL(debug) << id() << ": Found input fc: " << btaFc;
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning) << id() << " Could not cast input " << btaFc;
    return false;
  }
  boost::shared_ptr<cv::Mat> inImg;
  try {
    inImg = in.getMatPtr(in_img);
    BOOST_LOG_TRIVIAL(debug) << id() << ": Found input in_img: " << in_img;
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning) << id() << " Could not cast input " << in_img
                               << ", filter  " << id() << " not applied.";
    return true;
  }

  boost::shared_ptr<cv::Mat> ampl;
  try {
    ampl = in.getMatPtr(in_ampl);
    BOOST_LOG_TRIVIAL(debug) << id() << ": Found input in_ampl: " << in_ampl;
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning) << id() << " Could not cast input " << in_ampl
                               << ", filter  " << id() << " not applied.";
    return true;
  }

  boost::shared_ptr<std::vector<DetectedObject*> > blobs;
  try {
    blobs = boost::any_cast<boost::shared_ptr<std::vector<DetectedObject*> > >(
        out.getData(out_blobs));
    blobs->clear();
    BOOST_LOG_TRIVIAL(debug)
        << id() << ": Found output out_blobs: " << out_blobs;
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning) << id() << " Could not find object vector.";
    blobs.reset(new std::vector<DetectedObject*>);
    // return true;
  }
  blobs->clear();

  findBlobs(*inImg, *ampl, fc, *blobs);

  out.addData(out_blobs, blobs);

  cout << id() << ": SimpleBlobs saved into " << out_blobs
       << " size  =" << blobs->size() << endl;
  return true;
}

void SimpleBlobs::findBlobs(cv::Mat& img, cv::Mat& ampl, int fc,
                            std::vector<DetectedObject*>& detObj) {
  // BOOST_LOG_TRIVIAL(debug) << __FILE__ << ": " << __FUNCTION__;

  // Filter
  Mat m;
  
  if (img.type() == CV_32F) {
    m = img.clone();

    // Convert to 8bit
    double max, min;
    cv::minMaxLoc(m, &min, &max);
    m.convertTo(m, CV_8U, 255.0 / (max - min), -255.0 * min / (max - min));
  } else {
    m = img;
  }

  // Apply the optional morphology operation
  if (_morpho) {
    Mat element = getStructuringElement(
        MORPH_RECT, Size(2 * _morphoSize + 1, 2 * _morphoSize + 1),
        Point(_morphoSize, _morphoSize));
    morphologyEx(m, m, _morphoType, element, Point(-1, -1), _morphoIter);
  }

  if (dbg) {
     imshow("blobs to track ", m);
  }

  // Find the blob contours
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  findContours(m, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);

  cout << "contours: " << contours.size() << endl;

  // Shows all found contours
  if (dbg) {
    Mat imgCopy(img.size(), CV_8UC3, Scalar(0, 0, 0));
    RNG rng(12345);
    for (size_t i = 0; i < contours.size(); i++) {
      if (hierarchy[i][3] == -1) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                              rng.uniform(0, 255));
        drawContours(imgCopy, contours, i, color, -1, LINE_AA);
      }
    }
    imshow("detBlobs", imgCopy);
  }

  // accumulate blob meta-data
  DetectedObject* obj;
  Moments mu;
  Point2f mc;

  if (dbg) {
     cout << id() <<  " Blobs: # = " << contours.size() << endl;
  }

  for (size_t i = 0; i < contours.size(); i++) {
    mu = moments(contours[i]);
    mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
    
    if (mc.x != mc.x) {  // skip nan value areas
      // cout << "SKIP nan " << i << " " << mc << " " <<  endl;
      continue;
    }

    // Filter by size
    if (mu.m00 < _minSize) {
      continue;
    }
    // Filter if has internal holes
    if (_filterInternals && hierarchy[i][3] != -1) {
      continue;
    }

    // Create and object for each blob
    obj = new DetectedObject();
    obj->fc = fc;

    obj->massCenter = mc;

    obj->mo = mu;

    double hu[7];

    HuMoments(mu, hu);
    
    for(int i = 0; i < 7; i++) {
      obj->logHu[i] = -1 * copysign(1.0, hu[i]) * log10(abs(hu[i])); 
    }

    
    // Moving the massCenter if it is outside or in a hole
    if (pointPolygonTest(contours[i], obj->massCenter, false) < 0
        /* in case of a depth image, check distance
          || img.at<float>(obj->massCenter.y, obj->massCenter.x) < .05   */
    ) {
      obj->massCenter = contours[i][0];
    }

    obj->idx = i;
    obj->contour = contours[i];
    obj->contours.reset(new vector<vector<Point> >(contours));
    obj->hierarchy.reset(new vector<Vec4i>(hierarchy));
    obj->mo = mu;
    obj->size = contourArea(contours[i]);

    if (img.type() == CV_32F) {  // if img == depth image:
      obj->massCenterZ = img.at<float>(obj->massCenter);
      obj->massCenter3D = commons::pointTo3D(obj->massCenter, obj->massCenterZ);
    }

    // adding detected object to the output list
    if (obj->idx >= 0) detObj.push_back(obj);

    if (dbgShape) {
      DetectedObject* o = obj;
      cout << setprecision(3);
      cout << "BLB " << o->id << " " << o->idx << "\t" << (int)o->massCenter.x << "\t"
           << (int)o->massCenter.y << " " << (int)o->size
           << "\thu ";
      for (int i=0;i<7;i++) cout << " " << obj->logHu[i];
           //  << o->mo.m00 << "\t" << o->mo.m10 << "\t"
           //  << o->mo.m01 << "\t" << o->mo.m20 << "\t"
           //  << o->mo.m11 << "\t" << o->mo.m02 << "\t"
           //  << o->mo.m30 << "\t" << o->mo.m21 << "\t"
           //  << o->mo.m12 << "\t" << o->mo.m03 << "\t"

           //  << "mu\t" << o->mo.mu20 << "\t" << o->mo.mu11 << "\t" <<
           //  o->mo.mu02
           //  << "\t" << o->mo.mu30 << "\t" << o->mo.mu21 << "\t" << o->mo.mu12
           //  << "\t" << o->mo.mu03 << "\t"
           //  << "nu\t" << o->mo.nu20 << "\t" << o->mo.nu11 << "\t" <<
           //  o->mo.nu02
           //  << "\t" << o->mo.nu30 << "\t" << o->mo.nu21 << "\t" << o->mo.nu12
           //  << "\t" << o->mo.nu03
      cout  << endl;
    }
  }

  // shows blobs after filtering
  if (dbg) {
    Mat imgCopy(img.size(), CV_8UC3, Scalar(0, 0, 0));
    // img.convertTo(imgCopy,CV_GRAY2BGR);
    RNG rng(12345);
    for (size_t i = 0; i < detObj.size(); i++) {
      Scalar color =
          Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
      drawContours(imgCopy, *detObj[i]->contours, detObj[i]->idx, color, FILLED,
                   LINE_AA, *detObj[i]->hierarchy);
      circle(imgCopy, detObj[i]->massCenter, 5, Scalar(0, 255, 255), FILLED,
             LINE_AA);
    }
    imshow("detBlobs", imgCopy);
  }
}
