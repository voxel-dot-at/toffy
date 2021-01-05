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
#include "toffy/frame.hpp"

#include <boost/log/trivial.hpp>

using namespace toffy;

Frame::Frame() {}
Frame::Frame(const Frame& f) : data(f.data) {}

Frame::~Frame() { data.clear(); }

bool Frame::hasKey(std::string key) const {
  if (data.find(key) != data.end()) return true;
  return false;
}

boost::any Frame::getData(const std::string& key) const {
  // BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << ", key: " << key;
  boost::any out;
  if (data.find(key) != data.end()) {
    try {
      out = data.at(key);
    } catch (std::out_of_range& e) {
      BOOST_LOG_TRIVIAL(warning)
          << "Frame::getData(): Could not find key " << key;
    }
  } else {
    BOOST_LOG_TRIVIAL(info) << "Frame::getData(): Could not find key " << key;
  }
  return out;
}

void toffy::Frame::addData(std::string key, boost::any in) {
  data[key] = in;
  return;
}

bool toffy::Frame::removeData(std::string key) {
  if (data.find(key) != data.end()) {
    data.erase(data.find(key));
    return true;
  } else
    return false;
}

void Frame::clearData() { return data.clear(); }

/*
std::vector<std::string> Frame::keys() const
{
    std::vector<std::string> k(data.size());
    boost::container::flat_map< std::string, boost::any >::const_iterator it =
data.begin(); while (it!= data.end()) { k.push_back(it->first); it++;
    }
    return k;
}
*/
