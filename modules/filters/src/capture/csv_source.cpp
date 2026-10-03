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
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdio.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>

#include <opencv2/highgui.hpp>
#include <toffy/filter_helpers.hpp>
#include <toffy/io/csv_source.hpp>

using namespace std;
using namespace cv;
using namespace toffy::capturers;

const std::string CSVSource::id_name = "csvSource";  ///< Filter identifier

/*
 * N5 / P3-6: the pattern validator and the checked expansion helper moved to
 * toffy/filter_helpers.hpp when ExportCSV turned out to have the same defect -
 * two copies of a security-relevant parser drift. See finding N5, item P3-6.
 */
using toffy::validSequencePattern;
using toffy::formatPath;
CSVSource::CSVSource(): CapturerFilter(CSVSource::id_name, 0),
     width(160), height(120),
      _amplPattern("data/%05d_a.csv"),
      _depthPattern("data/%05d_d.csv"),
      _out_depth("depth"),
      _out_ampl("ampl"),
      // A24: neither had an initialiser, and filter() branches on useSequence.
      sequence(0), useSequence(false) {
      }

CSVSource::~CSVSource() {}

int CSVSource::loadConfig(const boost::property_tree::ptree& pt) {
  BOOST_LOG_TRIVIAL(debug) << " ------------ CSVSource::loadConfig() " << id();

  Filter::loadConfig(pt);

  width = pt.get<int>("options.width", width);
  height = pt.get<int>("options.height", height);
  const string amplPattern =
      pt.get<string>("options.amplPattern", _amplPattern);
  const string depthPattern =
      pt.get<string>("options.depthPattern", _depthPattern);
  _out_depth = pt.get<string>("outputs.depth", _out_depth);
  _out_ampl = pt.get<string>("outputs.ampl", _out_ampl);

  // N5 / P3-6: a pattern is a printf format string, so an invalid one is
  // undefined behaviour rather than a wrong file name. Diagnose it here and
  // keep the pattern that was already in use. The return value is reported but
  // is deliberately not fatal - a non-positive loadConfig() has no agreed
  // meaning yet (finding A23) and FilterBank::instantiateFilter does not check
  // it, so this cannot abort config loading.
  int errors = 0;
  std::string why;
  if (validSequencePattern(amplPattern, why)) {
    _amplPattern = amplPattern;
  } else {
    BOOST_LOG_TRIVIAL(error) << "CSVSource::" << __FUNCTION__
                             << ": rejecting options/amplPattern \"" << amplPattern
                             << "\" - " << why << "; keeping \"" << _amplPattern
                             << "\"";
    errors++;
  }
  if (validSequencePattern(depthPattern, why)) {
    _depthPattern = depthPattern;
  } else {
    BOOST_LOG_TRIVIAL(error) << "CSVSource::" << __FUNCTION__
                             << ": rejecting options/depthPattern \""
                             << depthPattern << "\" - " << why << "; keeping \""
                             << _depthPattern << "\"";
    errors++;
  }

  // A24: this assigned the flag into the *frame counter* (`sequence`) and left
  // `useSequence` - the member filter() actually branches on - uninitialised.
  // updateConfig() already read the same key into useSequence.
  useSequence = pt.get<bool>("options.sequence", useSequence);

  std::string pat = pt.get<string>("options.fcs", "");
  if (pat.length() > 0) {
          stringstream stream(pat);
          int fc;
          while (stream >>fc) {
                  fcs.push_back(fc);
                  cout << "fc: " << fc << endl;
          } 
  }
  BOOST_LOG_TRIVIAL(debug) << "CSVSource::loadConfig() configured to  " << width
                           << "x" << height << " " << _amplPattern << " "
                           << _depthPattern;
  return errors ? 0 : 1;
}

boost::property_tree::ptree CSVSource::getConfig() const {
    boost::property_tree::ptree pt;

    pt.put("options.width", width);
    pt.put("options.height", height);

    pt.put("options.amplPattern", _amplPattern);
    pt.put("options.depthPattern", _depthPattern);
    pt.put("outputs.depth", _out_depth);
    pt.put("outputs.ampl", _out_ampl);

    // A24: was `sequence`, the frame counter, so round-tripping a config wrote
    // the current playback position into the flag that enables playback.
    pt.put("options.sequence", useSequence);
    // @TODO export int array
    //    pt.put("options.fcs", fcs);

    return pt;
}

void CSVSource::updateConfig(const boost::property_tree::ptree& pt) {
          BOOST_LOG_TRIVIAL(debug) << " ------------ CSVSource::updateConfig() " << id();

  width = pt.get<int>("options.width", width);
  height = pt.get<int>("options.height", height);
  const string amplPattern =
      pt.get<string>("options.amplPattern", _amplPattern);
  const string depthPattern =
      pt.get<string>("options.depthPattern", _depthPattern);
  _out_depth = pt.get<string>("outputs.depth", _out_depth);
  _out_ampl = pt.get<string>("outputs.ampl", _out_ampl);

  // N5 / P3-6: same rule as loadConfig() - a runtime reconfigure must not be
  // able to install a format string that snprintf would misinterpret. The
  // previous, known-good pattern stays in use.
  std::string why;
  if (validSequencePattern(amplPattern, why)) {
    _amplPattern = amplPattern;
  } else {
    BOOST_LOG_TRIVIAL(error) << "CSVSource::" << __FUNCTION__
                             << ": rejecting options/amplPattern \"" << amplPattern
                             << "\" - " << why << "; keeping \"" << _amplPattern
                             << "\"";
  }
  if (validSequencePattern(depthPattern, why)) {
    _depthPattern = depthPattern;
  } else {
    BOOST_LOG_TRIVIAL(error) << "CSVSource::" << __FUNCTION__
                             << ": rejecting options/depthPattern \""
                             << depthPattern << "\" - " << why << "; keeping \""
                             << _depthPattern << "\"";
  }

  useSequence = pt.get<bool>("options.sequence", useSequence);

  std::string pat = pt.get<string>("options.fcs", "");
  if (pat.length() > 0) {
          stringstream stream(pat);
          int fc;
          while (stream >>fc) {
                  fcs.push_back(fc);
                  cout << "fc: " << fc << endl;
          } 
  }
  BOOST_LOG_TRIVIAL(debug) << "CSVSource::loadConfig() configured to  " << width
                           << "x" << height << " " << _amplPattern << " "
                           << _depthPattern << " seq? " << useSequence;
}

bool CSVSource::filter(const Frame& /*in*/, Frame& out) {
  BOOST_LOG_TRIVIAL(debug) << " ------------ CSVSource::filter() " << id();
  cv::waitKey(500);
  BOOST_LOG_TRIVIAL(debug) << " ------------ CSVSource::filter() " << id();

  matPtr ampl;
  matPtr depth;

  if (out.hasKey(_out_ampl)) {
    ampl = out.getMatPtr(_out_ampl);
  } else {
    // initialize ampl matrix ...:
    ampl.reset(new cv::Mat(height, width, CV_16U));
    out.addData(_out_ampl, ampl);
  }

  if (out.hasKey(_out_depth)) {
    depth = out.getMatPtr(_out_depth);
  } else {
    // initialize depth matrix ...:
    depth.reset(new cv::Mat(height, width, CV_32F));
    out.addData(_out_depth, depth);
  }

  // todo: handle 'file not found' as end of stream
  loadDepth(out, *depth);
  loadAmpl(out, *ampl);

  if (useSequence) {
        sequence++;
  }
  // todo: handle fc counts
  return true;
}

int CSVSource::connect() { return 0; }
int CSVSource::disconnect() { return 0; }
bool CSVSource::isConnected() { return true; }

int CSVSource::loadPath(const std::string& ) { return 1; }

void CSVSource::loadDepth(Frame& /*frame*/, Mat& depth) {
  char path[1024];
  if (!formatPath(path, sizeof(path), _depthPattern, sequence)) {
    BOOST_LOG_TRIVIAL(error) << "CSVSource: depthPattern \"" << _depthPattern
                             << "\" expands past " << sizeof(path)
                             << " characters; depth not updated";
    return;
  }

  cout << "loadD from " << path << endl;
  FILE* f = fopen(path, "r");
  if (!f) {
    cout << "COULD NOT OPEN " << path << " " << strerror(errno) << endl;
    return;
  }

  // N5 / P3-6: fscanf used to be unchecked, so a short or malformed CSV wrote
  // the still-uninitialised `val` into the rest of the image and the filter
  // reported success. Stop at the first value that is not there and say so;
  // the remaining pixels keep the content they already had.
  const long expected = static_cast<long>(width) * height;
  long read = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      float val = 0;
      if (fscanf(f, "%g;", &val) != 1) {
        BOOST_LOG_TRIVIAL(error) << "CSVSource: " << path << ": expected "
                                 << expected << " depth values, read " << read
                                 << "; the rest of the frame keeps its previous "
                                 << "content";
        fclose(f);
        return;
      }
      depth.at<float>(y,x) = val;
      read++;
    }
  }
  fclose(f);
}

void CSVSource::loadAmpl(Frame& /*frame*/, Mat& ampl) {
  char path[1024];
  if (!formatPath(path, sizeof(path), _amplPattern, sequence)) {
    BOOST_LOG_TRIVIAL(error) << "CSVSource: amplPattern \"" << _amplPattern
                             << "\" expands past " << sizeof(path)
                             << " characters; ampl not updated";
    return;
  }

  cout << "loadA from " << path << endl;
  FILE* f = fopen(path, "r");
  if (!f) {
    cout << "COULD NOT OPEN " << path << " " << strerror(errno) << endl;
    return;
  }

  // N5 / P3-6: see loadDepth().
  const long expected = static_cast<long>(width) * height;
  long read = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int val = 0;
      if (fscanf(f, "%d;", &val) != 1) {
        BOOST_LOG_TRIVIAL(error) << "CSVSource: " << path << ": expected "
                                 << expected << " ampl values, read " << read
                                 << "; the rest of the frame keeps its previous "
                                 << "content";
        fclose(f);
        return;
      }
      ampl.at<short>(y,x) = val;
      read++;
    }
  }
  fclose(f);
}
