/*
   Copyright 2021 Simon Vogl <svogl@voxel.at>

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

#include <boost/any.hpp>

#include <opencv2/core.hpp>

#include "toffy/filter_helpers.hpp"
#include "toffy/viewers/exportcsv.hpp"

using namespace toffy;
using namespace cv;

/*
 * N5 / P3-6: shared with CSVSource, which had the same defect - see
 * toffy/filter_helpers.hpp.
 */
using toffy::validSequencePattern;
using toffy::formatPath;

int ExportCSV::loadConfig(const boost::property_tree::ptree &pt) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__
                           << "(const boost::property_tree::ptree& pt)";
  const boost::property_tree::ptree &conf = pt.get_child(this->type());

  loadGlobals(conf);

  updateConfig(conf);

  return true;
}

void ExportCSV::updateConfig(const boost::property_tree::ptree &pt) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

  using namespace boost::property_tree;

  Filter::updateConfig(pt);

  const std::string filePattern =
      pt.get<std::string>("options.pattern", _filePattern);
  _seq = pt.get<bool>("options.sequence", _seq);
  _skip0s = pt.get<bool>("options.skipZeroes", _skip0s);
  _fc = pt.get<std::string>("options.fc", _fc);

  _in = pt.get<std::string>("inputs.img", _in);

  // N5 / P3-6: options/pattern is handed to snprintf as its *format* argument,
  // so a pattern that is not a valid format for one int is undefined behaviour,
  // not a wrong file name -Wformat cannot see (`%s` reads a pointer out of the
  // stack slot that holds the counter). Validate here, once, and keep the
  // pattern that was already working when the new one is not usable.
  std::string why;
  if (validSequencePattern(filePattern, why)) {
    _filePattern = filePattern;
  } else {
    LOG(error) << "rejecting options/pattern \"" << filePattern << "\" - "
               << why << "; keeping \"" << _filePattern << "\"";
  }

  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " fp " << _filePattern;
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " fc " << _fc;

}

boost::property_tree::ptree ExportCSV::getConfig() const {
  boost::property_tree::ptree pt;

  pt = Filter::getConfig();

  pt.put("options.pattern", _filePattern);
  pt.put("options.path", _path);
  pt.put("options.sequence", _seq);
  pt.put("options.frameCounter", _fc);
  pt.put("options.skipZeroes", _skip0s);

  pt.put("inputs.img", _in);

  return pt;
}

static void saveMatCSV(const std::string &fileName, const Mat& mat, bool skipZeroes);

bool ExportCSV::filter(const Frame &in, Frame &) {
  LOG(debug) << __FUNCTION__ << " " << id();
  matPtr input;
  try {
    input = boost::any_cast<matPtr>(in.getData(_in));
  } catch (const boost::bad_any_cast &) {
    LOG(warning) << "Could not cast input " << _in << ", filter  "
                               << id() << " not applied.";
    return false;
  }

  // N5 / P3-6: the buffer used to be a VLA sized `_filePattern.length() + 64`,
  // i.e. sized from a config string, and every expansion into it was unchecked:
  // an expansion that did not fit was opened as if it were the intended file.
  // 4096 is PATH_MAX on Linux; the pattern is validated at config time, so the
  // only remaining failure is a path that is simply too long, and that is now
  // reported instead of silently truncating.
  char path[4096];
  if (_seq) {
    if (!formatPath(path, sizeof(path), _filePattern, _cnt)) {
      // Deliberately not `return false`: a false from a filter aborts the whole
      // bank and skips its ready.post() (FilterBank::filter), which is a bigger
      // decision than this item. The error is reported and nothing is written.
      LOG(error) << "pattern \"" << _filePattern << "\" expands past "
                 << sizeof(path) << " characters; nothing written";
      return true;
    }
    _cnt++;

  } else {
    if (_fc.length()) {
      const int fc = static_cast<int>(in.getUInt(_fc));
      if (!formatPath(path, sizeof(path), _filePattern, fc)) {
        LOG(error) << "pattern \"" << _filePattern << "\" expands past "
                   << sizeof(path) << " characters; nothing written";
        return true;
      }
    } else {
      LOG(warning) << "no suitable file pattern found";
      // The pattern is used as a literal file name here, which is why the
      // format string is a literal too - the config string never reaches
      // snprintf as a format on this path.
      const int n = snprintf(path, sizeof(path), "%s", _filePattern.c_str());
      if (n < 0 || static_cast<std::size_t>(n) >= sizeof(path)) {
        LOG(error) << "file name \"" << _filePattern << "\" is longer than "
                   << sizeof(path) << " characters; nothing written";
        return true;
      }
    }
  }
  LOG(info) << " saving to " << path ;
  saveMatCSV(std::string(path), *input, _skip0s);

  return true;
}

static void saveMatCSV(const std::string &fileName, const Mat& mat, bool skipZeroes) {
  using namespace std;
  ofstream of(fileName, std::ofstream::out);

  uint32_t j = 0;
  switch (mat.type()) {
    case CV_8UC1:
      for (int y = 0; y < mat.rows; y++) {
        for (int x = 0; x < mat.cols; x++) {
          of << mat.at<char>(y, x) << ";";
          j++;
        }
        of << '\n';
      }
      break;
    case CV_16UC1:
      for (int y = 0; y < mat.rows; y++) {
        for (int x = 0; x < mat.cols; x++) {
          of << mat.at<unsigned short>(y, x) << ";";
          j++;
        }
        of << '\n';
      }
      break;
    case CV_16SC1:
      for (int y = 0; y < mat.rows; y++) {
        for (int x = 0; x < mat.cols; x++) {
          of << mat.at<short>(y, x) << ";";
          j++;
        }
        of << '\n';
      }
      break;
    case CV_32SC1:
      for (int y = 0; y < mat.rows; y++) {
        for (int x = 0; x < mat.cols; x++) {
          of << mat.at<int>(y, x) << ";";
          j++;
        }
        of << '\n';
      }
      break;
    case CV_32FC1:
      for (int y = 0; y < mat.rows; y++) {
        for (int x = 0; x < mat.cols; x++) {
          of << mat.at<float>(y, x) << ";";
          j++;
        }
        of << '\n';
      }
      break;
    case CV_64FC1:
      for (int y = 0; y < mat.rows; y++) {
        for (int x = 0; x < mat.cols; x++) {
          of << mat.at<double>(y, x) << ";";
          j++;
        }
        of << '\n';
      }
      break;
    default:
      BOOST_LOG_TRIVIAL(warning)
          << "no suitable mat data conversion for " 
          << mat.type() << " "
          << " type " << typeToString(mat.type())  
          << " depth " << depthToString(mat.depth());
  }
  of.close();
}
