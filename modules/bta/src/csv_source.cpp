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
#include <stdio.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>

#include <opencv2/highgui.hpp>
#include <toffy/io/csv_source.hpp>

using namespace std;
using namespace cv;
using namespace toffy::capturers;

const std::string CSVSource::id_name = "csvSource";  ///< Filter identifier

CSVSource::CSVSource(): CapturerFilter(CSVSource::id_name, 0),
     width(160), height(120), 
      _amplPattern("data/%05d_a.csv"),
      _depthPattern("data/%05d_d.csv"),
      _out_depth("depth"),
      _out_ampl("ampl") {
      }

CSVSource::~CSVSource() {}

int CSVSource::loadConfig(const boost::property_tree::ptree& pt) {
  BOOST_LOG_TRIVIAL(debug) << " ------------ CSVSource::loadConfig() " << id();
  const boost::property_tree::ptree& bta = pt.get_child(type());

  Filter::loadConfig(pt);

  width = pt.get<int>("options.width", width);
  height = pt.get<int>("options.height", height);
  _amplPattern = pt.get<string>("options.amplPattern", _amplPattern);
  _depthPattern = pt.get<string>("options.depthPattern", _depthPattern);
  _out_depth = pt.get<string>("outputs.depth", _out_depth);
  _out_ampl = pt.get<string>("outputs.ampl", _out_ampl);

  sequence = pt.get<bool>("options.sequence", sequence);

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
}

boost::property_tree::ptree CSVSource::getConfig() const {}

void CSVSource::updateConfig(const boost::property_tree::ptree& pt) {
          BOOST_LOG_TRIVIAL(debug) << " ------------ CSVSource::updateConfig() " << id();

  width = pt.get<int>("options.width", width);
  height = pt.get<int>("options.height", height);
  _amplPattern = pt.get<string>("options.amplPattern", _amplPattern);
  _depthPattern = pt.get<string>("options.depthPattern", _depthPattern);
  _out_depth = pt.get<string>("outputs.depth", _out_depth);
  _out_ampl = pt.get<string>("outputs.ampl", _out_ampl);

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

bool CSVSource::filter(const Frame& in, Frame& out) {
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

int CSVSource::loadPath(const std::string& newPath) {}

void CSVSource::loadDepth(Frame& frame, Mat& depth) {
  char path[1024];
  snprintf(path, sizeof(path), _depthPattern.c_str(), sequence);

  cout << "loadD from " << path << endl;
  FILE* f = fopen(path, "r");
  if (!f) {
    cout << "COULD NOT OPEN " << path << " " << strerror(errno) << endl;
    return;
  }

  for (int y = 0; y < height; y++) {
    int x=0;
    float val;
    // cout << y << "\t"; 
    for (int x = 0; x < width; x++) {
      fscanf(f, "%g;", &val);
      // cout << val << " "; 
      depth.at<float>(y,x) = val;
    }
    // cout <<  endl; 
  }
  fclose(f);
}

void CSVSource::loadAmpl(Frame& frame, Mat& ampl) {
  char path[1024];
  snprintf(path, sizeof(path), _amplPattern.c_str(), sequence);

  cout << "loadA from " << path << endl;
  FILE* f = fopen(path, "r");
  if (!f) {
    cout << "COULD NOT OPEN " << path << " " << strerror(errno) << endl;
    return;
  }

  for (int y = 0; y < height; y++) {
    int x=0;
    int val;
    // cout << y << "\t"; 
    for (int x = 0; x < width; x++) {
      fscanf(f, "%d;", &val);
      // cout << val << " "; 
      ampl.at<short>(y,x) = val;
    }
    // cout <<  endl; 
  }
  fclose(f);        
}
