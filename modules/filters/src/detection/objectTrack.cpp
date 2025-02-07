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
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>
#include <sstream>

#if OCV_VERSION_MAJOR >= 3
#include <opencv2/highgui.hpp>
#else
#include <opencv2/highgui/highgui.hpp>
#endif

#include <toffy/detection/detectedObject.hpp>
#include <toffy/detection/objectTrack.hpp>

using namespace toffy;
using namespace toffy::detection;
// using namespace cv;
using namespace std;

#ifdef CM_DEBUG
static const bool dbg = true;
#else
static const bool dbg = false;
#endif

std::size_t ObjectTrack::_filter_counter = 1;
const std::string ObjectTrack::id_name = "objectTrack";

ObjectTrack::ObjectTrack()
    : Filter(ObjectTrack::id_name, _filter_counter),
      _in_img("img"),
      _in_vec("objects"),
      _out_vec("dect"),
      cnt(0),
      numObjects(0),
      updateScript("updateCount.sh") {
  _filter_counter++;
}

boost::property_tree::ptree ObjectTrack::getConfig() const {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();
  boost::property_tree::ptree pt;

  pt = Filter::getConfig();
  // pt.put("options.max_x", _max_x);

  pt.put("inputs.img", _in_img);
  pt.put("inputs.vec", _in_vec);
  pt.put("outputs.vec", _out_vec);

  return pt;
}

void ObjectTrack::updateConfig(const boost::property_tree::ptree& pt) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

  using namespace boost::property_tree;

  Filter::updateConfig(pt);

  //_max_x = pt.get<float>("options.max_x",_max_x);

  _in_img = pt.get<string>("inputs.img", _in_img);
  _in_vec = pt.get<string>("inputs.vec", _in_vec);
  _out_vec = pt.get<string>("outputs.vec", _out_vec);

  updateScript = pt.get<string>("options.script", updateScript);
}

bool ObjectTrack::filter(const Frame& in, Frame& out) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

  toffy::Filter::setLoggingLvl();

  // List of detected objects
  BOOST_LOG_TRIVIAL(debug) << "Getting _in_vec: " << _in_vec;
  boost::shared_ptr<DetectedObjects > detObjs;
  try {
    detObjs = boost::any_cast<
        boost::shared_ptr<DetectedObjects > >(
        in.getData(_in_vec));
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning)
        << "Could not find object vector: " << _in_vec << ". " << id();
    return true;
  }

  BOOST_LOG_TRIVIAL(debug) << "Got _in_vec(" << _in_vec
                           << "): " << detObjs->size();

  // TODO Where come img from?, why we mask it with the objects.
  BOOST_LOG_TRIVIAL(debug) << "Getting got _in_img: " << _in_img;
  matPtr img;
  try {
    img = in.getMatPtr(_in_img);
  } catch (const boost::bad_any_cast&) {
    BOOST_LOG_TRIVIAL(warning)
        << "Could not cast input " << _in_img << ", filter  " << id()
        << " does not show objects.";
  }

  // Define zones!!!
  cv::Rect z1(0, 0, img->cols, (img->rows / 2) - 1);

  cv::Rect z2(0, 0, img->cols, (img->rows / 2) - 1);

  cv::Mat color;

  double max, min;
  minMaxIdx(*img, &min, &max);
  img->convertTo(color, CV_8U, 255.0 / (max - min), -255.0 * min / (max - min));
  cvtColor(color, color, cv::COLOR_GRAY2RGB);

  vector<detection::DetectedObject*>::iterator objDetIter = detObjs->begin();
  while (objDetIter != detObjs->end()) {
    detection::DetectedObject* obj = *objDetIter;
    if (obj->record->empty()) {
      objDetIter++;
      continue;
    }

    if (z1.contains(obj->firstCenter)) {
      if (z1.contains(obj->massCenter))
        BOOST_LOG_TRIVIAL(debug) << "IN ZONE 1";
      else {
        obj->firstCenter = obj->massCenter;
        BOOST_LOG_TRIVIAL(debug) << "Change to zone 2";
        cnt++;
        publishCount();
      }
    } else {
      if (!z1.contains(obj->massCenter))
        BOOST_LOG_TRIVIAL(debug) << "IN ZONE 2";
      else {
        obj->firstCenter = obj->massCenter;
        BOOST_LOG_TRIVIAL(debug) << "Change to zone 1";
        cnt--;
        publishCount();
      }
    }

    BOOST_LOG_TRIVIAL(debug) << "Old: " << obj->record->back()->massCenter3D;
    BOOST_LOG_TRIVIAL(debug) << "New: " << obj->massCenter3D;
    double dis =
        cv::norm(obj->massCenter3D - obj->record->front()->massCenter3D);
    BOOST_LOG_TRIVIAL(debug) << "dis: " << dis;
    boost::posix_time::time_duration time = obj->record->front()->ts - obj->ts;
    BOOST_LOG_TRIVIAL(debug) << "time: " << time.seconds();
    double speed = dis / time.seconds();
    BOOST_LOG_TRIVIAL(debug) << "speed: " << speed;

    cv::arrowedLine(color, obj->record->back()->massCenter, obj->massCenter,
                    CV_RGB(255, 255, 0));

    cv::line(color, cv::Point(z1.x, z1.height), z1.br(), CV_RGB(255, 0, 0), 1);

    cv::putText(color, boost::str(boost::format("%.4f") % speed) + " m/s",
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                CV_RGB(255, 255, 0));

    cv::putText(color, boost::lexical_cast<std::string>(cnt), cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 0));

    objDetIter++;

    cv::namedWindow("objectTrack", cv::WINDOW_NORMAL);
    imshow("objectTrack", color);
  }
  if (numObjects != detObjs->size()) publishCount();

  numObjects = detObjs->size();

  return true;
}

void ObjectTrack::publishCount() {
  std::stringstream sb;
  sb << updateScript << " " << cnt << " " << numObjects << "&";

  BOOST_LOG_TRIVIAL(debug) << "publ: " << sb.str();

  system(sb.str().c_str());
}
