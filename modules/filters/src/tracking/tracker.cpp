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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/log/trivial.hpp>
#include <iostream>

#if CV_VERSION_MAJOR >= 4 || OCV_VERSION_MAJOR >= 3
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#else
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

#include "toffy/tracking/tracker.hpp"

using namespace toffy;
using namespace toffy::tracking;
using namespace cv;
using namespace std;

#ifdef CM_DEBUG
const bool dbg = true;
#else
const bool dbg = false;
#endif

std::size_t Tracker::_filter_counter = 1;
const std::string Tracker::id_name = "tracker";

Tracker::Tracker()
    : Filter(Tracker::id_name, _filter_counter),
      _in_vec("dect"),
      _in_fc("fc"),
      _in_img("img"),
      _out_img(_in_img),
      _out_objects("objects"),
      _out_count("count"),
      maxMergeDistance(20.),
      _render_image(true) {
  _filter_counter++;
}

void Tracker::updateConfig(const boost::property_tree::ptree &pt) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

  using namespace boost::property_tree;

  Filter::updateConfig(pt);

  _in_vec = pt.get<string>("inputs.vec", _in_vec);
  _in_fc = pt.get<string>("inputs.fc", _in_fc);
  _in_img = pt.get<string>("inputs.img", _in_img);

  _out_img = pt.get<string>("outputs.img", _out_img);
  _out_objects = pt.get<string>("outputs.objects", _out_objects);
  _out_count = pt.get<string>("outputs.count", _out_count);

  maxMergeDistance =
      pt.get<double>("options.maxMergeDistance", maxMergeDistance);
  _render_image = pt.get<bool>("options.renderImage", _render_image);
}

boost::property_tree::ptree Tracker::getConfig() const {
  boost::property_tree::ptree pt;

  pt = Filter::getConfig();
  // pt.put("options.max_x", _max_x);

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

bool Tracker::filter(const Frame &in, Frame &out) {

  toffy::Filter::setLoggingLvl();

  // Get list of detected blobs
  boost::shared_ptr<std::vector<detection::DetectedObject *> > blobs;
  try {
    blobs = boost::any_cast<
        boost::shared_ptr<std::vector<detection::DetectedObject *> > >(
        in.getData(_in_vec));
  } catch (const boost::bad_any_cast &) {
    BOOST_LOG_TRIVIAL(warning)
        << id() << " Could not find object vector: " << _in_vec
        << ". Nothing to do!";
    return true;
  }

  BOOST_LOG_TRIVIAL(debug) << id() << " Got _in_vec size: " << blobs->size();

  // Current frame counter
  try {
    _fc = in.getUInt(_in_fc);
  } catch (const boost::bad_any_cast &) {
    BOOST_LOG_TRIVIAL(warning) << id() << " Could not find framecounter. ";
    _fc = -1;
  }

  // Current Timestamp
  try {
    _ts = in.getUInt("ts");
  } catch (const boost::bad_any_cast &) {
    BOOST_LOG_TRIVIAL(warning) << id() << " Could not find Timestamp. ";
    _ts = -1;
  }

  BOOST_LOG_TRIVIAL(debug) << id() << " filter() fc " << _fc << " ts " << _ts ;

  // List of tracked objects
  boost::shared_ptr<std::vector<detection::DetectedObject *> > tracked;
  try {
    tracked = boost::any_cast<
        boost::shared_ptr<std::vector<detection::DetectedObject *> > >(
        out.getData(_out_objects));
  } catch (const boost::bad_any_cast &) {
    BOOST_LOG_TRIVIAL(warning)
        << id() << " Could not find object vector. Creating a new one";
    tracked.reset(new std::vector<detection::DetectedObject *>);
  }

  vector<detection::DetectedObject *> newState;  // the new state..

  // 1. compare all old objects against the candidates, extract best matches
  // 2. for all candidates that could not be matched, add them if applicable
  // 3. eliminate all old detObjs ( not seen >5 frames)
  // 4. save the new state.

  ///// 1.: compare all old objects against the candidates, extract best matches

  BOOST_LOG_TRIVIAL(debug) << id() << " detObjs: " << detObjs.size();
  BOOST_LOG_TRIVIAL(debug) << id() << " blobs: " << blobs->size();

  // compare the list of already tracked objects against incoming blobs.
  // finds the nearest candidate; if it is near enough, consider it as the new position
  // else we might have found a new blob to track: 
  vector<detection::DetectedObject *>::iterator objDetIter = detObjs.begin();
  while (objDetIter != detObjs.end()) {
    detection::DetectedObject *trackedObj = *objDetIter;
    detection::DetectedObject *blob = NULL;

    // Compare the object massCenter distances, find best match
    int candidate = -1;                // Flag for potential match
    double minDis = maxMergeDistance + .01;  // Distance to merge
    for (size_t j = 0; j < blobs->size(); j++) {
      blob = (*blobs)[j];
      if (!blob) continue;  // For removed blob from input list

      double ndis = norm(trackedObj->massCenter - blob->massCenter);

      // BOOST_LOG_TRIVIAL(debug) << id() << "  (1): " << trackedObj->id << " " << blob->id << " dist " << ndis;

      if (ndis < minDis) {
        minDis = ndis;
        candidate = j;
      }
    }
    // if no near candidate has been found assume a new obj (step 2).
    if (minDis > maxMergeDistance) {
      candidate = -1;
      blob = 0;
    }

    if (candidate >= 0) {
      blob = (*blobs)[candidate];
      blob->size = contourArea(blob->contour);

      // BOOST_LOG_TRIVIAL(debug)
      //     << "Found candidate object id = " << blob->id << "\t" << minDis
      //     << "\t" << blob->size << "\t" << blob->contour.size() << "\t"
      //     << blob->massCenter << " for " << trackedObj->id;

      // found a match, update obj data:
      trackedObj->update(*blob);

      // TODO We destroy the input blob list!!!
      // remove blob from candidates & delete
      delete blob;
      (*blobs)[candidate] = NULL;

      // move obj to new state
      newState.push_back(trackedObj);
      objDetIter = detObjs.erase(objDetIter);
    } else {
      objDetIter++;
    }
  }

  /////// 2.: add all valid new candidates:
  vector<detection::DetectedObject *>::iterator candIter = blobs->begin();
  while (candIter != blobs->end()) {
    detection::DetectedObject *blob = *candIter;
    if (blob == NULL) {
      candIter++;
      continue;
    }
    // BOOST_LOG_TRIVIAL(debug) << id() << "  (2): " << blob->id << " c " << blob->contour.size();

    // TODO Parameter?
    if (blob->contour.size() < 4) {
        // BOOST_LOG_TRIVIAL(debug) << id() << "  (2): ignoring " << blob->id;

      delete blob;
      blob = NULL;
      // candIter = candidates.erase(candIter);
    } else {
      // init new detected object
      // BOOST_LOG_TRIVIAL(debug) << id() << "  (2):   new trk " << blob->id << " " << nextId;
      blob->id = this->nextId++;
      blob->fc = _fc;
      blob->ts = boost::posix_time::from_time_t(_ts);
      blob->init();
      /*blob->first_fc = blob->fc;
      blob->first_cts = blob->cts;
      blob->first_ts = blob->ts;
      blob->firstCenter = blob->massCenter;
      blob->size = contourArea(blob->contour);
      blob->record.reset(new boost::circular_buffer<detection::DetectedObject*
      >(10));*/

      newState.push_back(blob);
      // BOOST_LOG_TRIVIAL(debug) << id() << "  (2):       trk " << blob->id;
    }
    candIter++;
  }
  // TODO do not delete input blobs!
  blobs->clear();

  /////// 3. eliminate all old detObjs ( not seen >5 frames)
  while (objDetIter != detObjs.end()) {
    detection::DetectedObject *obj = *objDetIter;

    // TODO Parameter!
    if (abs(_fc - obj->fc) > 5) {
      // BOOST_LOG_TRIVIAL(debug) << id() << "  (3):       kill " << obj->id;
      delete obj;
    } else {
      newState.push_back(obj);
    }
  }
  // at this point, the contents of detObjs is not touched any more and will be
  // overwritten.

  ///// 4. save the new state.
  detObjs = newState;
  out.addData(_out_count, detObjs.size());

  *tracked = newState;

  out.addData(_out_objects, tracked);

  // Debug, shows image with tracked objects
  if (_render_image) {
    boost::shared_ptr<cv::Mat> img;
    try {
      img = in.getMatPtr(_in_img);
    } catch (const boost::bad_any_cast &) {
      BOOST_LOG_TRIVIAL(warning)  << id()
          << " Could not cast input " << _in_img << ", filter  " << id()
          << " does not show objects.";
      return true;
    }

    BOOST_LOG_TRIVIAL(debug) << id() << " Getting got _out_img: " << _out_img;
    boost::shared_ptr<cv::Mat> img_out;
    try {
      img_out = in.getMatPtr(_out_img);
    } catch (const boost::bad_any_cast &) {
      BOOST_LOG_TRIVIAL(info)  << id() << " Creating outputs.img : " << _out_img;
      img_out.reset(new Mat());
      out.addData(_out_img, img_out);
    }

    img->copyTo(*img_out);
    showObjects(*img_out);
  }
  return true;
}

void Tracker::showObjects(cv::Mat &depth) {
  double max, min;
  minMaxIdx(depth, &min, &max);
  depth.convertTo(depth, CV_8U, 255.0 / (max - min),
                  -255.0 * min / (max - min));
  cvtColor(depth, depth, COLOR_GRAY2RGB);

  // BOOST_LOG_TRIVIAL(debug) << "showObjects " << detObjs.size();
  for (size_t i = 0; i < detObjs.size(); i++) {
    if (detObjs[i]->first_fc == (int)_fc) {
      circle(depth, detObjs[i]->massCenter, 4, detObjs[i]->color);
      continue;
    }
    try {
      if (detObjs[i]->fc == (int)_fc) {
        vector<vector<Point> > contours;
        contours.push_back(detObjs[i]->contour);
        drawContours(depth, contours, 0, detObjs[i]->color, 1, LINE_AA,
                     noArray(), 0, Point());
        circle(depth, detObjs[i]->massCenter, 2, detObjs[i]->color);
      }
    } catch (std::exception &e) {
      LOG(warning)  << id() << " Auweh - could not paint detObjs.s=" << detObjs.size()
                   << " i= " << i;
    }
  }

  if (dbg) cv::imshow(id() + ".objects", depth);

  return;
}
