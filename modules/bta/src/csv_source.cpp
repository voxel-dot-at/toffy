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
#include <boost/algorithm/string/trim.hpp>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>
#include <fstream>
#include <toffy/io/csv_source.hpp>

using namespace std;
using namespace toffy::capturers;

const std::string CSVSource::id_name = "csvSource";  ///< Filter identifier

CSVSource::CSVSource()
    : width(160), height(120), 
      _amplPattern("data/%05d_a.csv"),
      _depthPattern("data/%05d_d.csv"),
      _out_depth("depth"),
      _out_ampl("ampl") {}

CSVSource::~CSVSource() {}

int CSVSource::loadConfig(const boost::property_tree::ptree& pt) {
  BOOST_LOG_TRIVIAL(debug) << " ------------ CSVSource::loadConfig() " << id();
  const boost::property_tree::ptree& bta = pt.get_child(type());

  Filter::loadConfig(pt);

  //     std::string _amplPattern, _distPattern, _out_depth, _out_ampl,
  //         _out_mf, _out_it,
  //         _out_fc, _out_ts,
  //         _out_mt, _out_lt, _out_gt

  width = pt.get<int>("options.width", width);
  height = pt.get<int>("options.height", width);
  _amplPattern = pt.get<string>("options.amplPattern", _amplPattern);
  _depthPattern = pt.get<string>("options.depthPattern", _depthPattern);
  _out_depth = pt.get<string>("outputs.depth", _out_depth);
  _out_ampl = pt.get<string>("outputs.ampl", _out_ampl);

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
          BOOST_LOG_TRIVIAL(debug) << " ------------ CSVSource::filter() " << id();

}

bool CSVSource::filter(const Frame& in, Frame& out) {
  BOOST_LOG_TRIVIAL(debug) << " ------------ CSVSource::filter() " << id();
  loadDepth(out);
  loadAmpl(out);
}

int CSVSource::connect() {}
int CSVSource::disconnect() {}
bool CSVSource::isConnected() {}

int CSVSource::loadPath(const std::string& newPath) {}

void CSVSource::loadDepth(Frame& f) {
        
}

void CSVSource::loadAmpl(Frame& f) {
        
}
