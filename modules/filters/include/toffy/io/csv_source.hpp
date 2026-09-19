/*
   Copyright 2012-2021 Simon Vogl <svogl@voxel.at> VoXel Interaction Design - www.voxel.at

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
#pragma once

#include <toffy/capture/capturerFilter.hpp>

namespace toffy {
namespace capturers {

/**
 * @brief reads in csv files
 * @ingroup Capturers
 *
 */
class CSVSource : public CapturerFilter {

public:
    static const std::string id_name; ///< Filter identifier

    CSVSource();

    virtual ~CSVSource();

    virtual int loadConfig(const boost::property_tree::ptree& pt);

    virtual boost::property_tree::ptree getConfig() const;

    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

    virtual int connect();
    virtual int disconnect();
    virtual bool isConnected();

    virtual bool playback() const {return true;}

    virtual int loadPath(const std::string &newPath);

    static Filter* creator() { return new CSVSource();}

private:
    // frame size data
    int width, height;
    std::string _amplPattern, _depthPattern, _out_depth, _out_ampl,
        _out_mf, _out_it,
        _out_fc, _out_ts,
        _out_mt, _out_lt, _out_gt;
std::vector<int> fcs;
int sequence; 
bool useSequence;
  void loadDepth(Frame& frame, cv::Mat& depth);
  void loadAmpl(Frame& frame, cv::Mat& ampl);
};

} // capturers
} // toffy
