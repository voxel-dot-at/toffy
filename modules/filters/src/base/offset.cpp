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

#include <fstream>
#include <iostream>
#include <string>
#include <limits.h>

#include <boost/log/trivial.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/case_conv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "toffy/base/offset.hpp"
#include "toffy/filter_helpers.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;


std::size_t OffSet::_filter_counter = 1;
const std::string OffSet::id_name = "offset";

OffSet::OffSet()
    : Filter(OffSet::id_name, _filter_counter),
      _sumValue(0.0),
      _mulValue(1.0),
      _in_img("img") {
  _filter_counter++;
}

/*
int OffSet::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const
boost::property_tree::ptree& pt)"; const boost::property_tree::ptree& tree =
pt.get_child(this->type());

    loadGlobals(tree);

    updateConfig(tree);

    return true;
}
*/

void OffSet::updateConfig(const boost::property_tree::ptree &pt) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

  using namespace boost::property_tree;

  Filter::updateConfig(pt); // sets log level and other common settings

  _in_img = pt.get<string>("inputs.img", _in_img);

  _sumValue = pt.get<float>("options.sumValue", _sumValue);
  _mulValue = pt.get<float>("options.mulValue", _mulValue);

  _out_img = pt.get<string>("outputs.img", _out_img);

  try {
    BOOST_FOREACH (const ptree::value_type &v, pt.get_child("options.roi")) {
      Rect roi(0, 0, 0, 0);
      roi.x = v.second.get<int>("x", roi.x);
      roi.y = v.second.get<int>("y", roi.y);
      roi.width = v.second.get<int>("width", roi.width);
      roi.height = v.second.get<int>("height", roi.height);
      _rois.push_back(roi);
    }
  } catch (const std::exception &ex) {
    BOOST_LOG_TRIVIAL(debug) << "no roi found. Using full image.";
  }
}

bool OffSet::filter(const Frame &in, Frame &out) const {
  LOG(debug) << __FUNCTION__;
  using namespace boost::posix_time;

  auto start = microsec_clock::local_time();
  boost::posix_time::time_duration diff;

  if (_rois.size() == 0) {
    LOG(debug) << "depth found";
    matPtr d = in.getMatPtr(_in_img);
    matPtr o;

    if (out.hasKey(_out_img))
      o = boost::any_cast<matPtr>(in.getData(_out_img));
    else {
      o.reset(new Mat());
      out.addData(_out_img, o);
    }

    *o = *d + _sumValue;
    *o *= _mulValue;

  } else {
    for (size_t i = 0; i < _rois.size(); i++) {
      matPtr d = in.getMatPtr(_in_img);
      Mat roi((*d)(_rois[i]));
      roi += _sumValue;
      roi *= _mulValue;
    }
  }

  diff = boost::posix_time::microsec_clock::local_time() - start;
  BOOST_LOG_TRIVIAL(debug) << "OffSet::filter: " << diff.total_microseconds();

  return true;
}

void OffSet::addRoi(cv::Rect newRoi) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
  _rois.push_back(newRoi);
}

boost::property_tree::ptree OffSet::getConfig() const {
  boost::property_tree::ptree pt, opt;

  pt = Filter::getConfig();

  opt.put("sumValue", _sumValue);
  opt.put("mulValue", _mulValue);
  pt.add_child("options", opt);
  opt.clear();

  opt.put("img", _in_img);
  pt.add_child("inputs", opt);
  opt.clear();

  opt.put("img", _out_img);
  pt.add_child("outputs", opt);

  return pt;
}
