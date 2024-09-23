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

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <toffy/capture/capturerFilter.hpp>
#include <toffy/cam/cameraParams.hpp>

class BtaWrapper; // forward declaration of the lower-layer sensor wrapper class

namespace toffy {
namespace capturers {

/**
 * @brief Bluetechnix BtaTofApi wrapper filter
 * @ingroup Capturers
 *
 * For detailled information see \ref bta_page description page
 *
 */
class Bta : public CapturerFilter {

    BtaWrapper *sensor; ///< Sensor
    bool bta_stream; ///< Flag indicates if using btastream files
    static std::size_t _filter_counter; ///< Internal class counter

public:
    static const std::string id_name; ///< Filter identifier

    /**
     * @brief Bta
     */
    Bta();

    /**
     * @brief ~Bta
     */
    virtual ~Bta();

    /**
     * @brief Accesss to the camera wrapper
     * @return BtaWrapper *
     *
     */
    BtaWrapper * getSensor() const {return sensor;}

    virtual int loadConfig(const boost::property_tree::ptree& pt);

    virtual boost::property_tree::ptree getConfig() const;

    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

    virtual int connect();
    virtual int disconnect();
    virtual bool isConnected();

    virtual bool playback() const {return CapturerFilter::playback();}
    virtual void playback(const bool &pb);

    virtual int loadPath(const std::string &newPath);
    virtual void save(const bool &save);
    virtual void savePath(const std::string &newPath);

    static Filter* creator() { return new Bta();}

private:
    // frame size data
    int distsSize = 0;
    int width = 0, height = 0;
    bool dynOutputs = true; // dynamically map output depending on channels present

    float globalOfs = 0.f;

    uint32_t eth0Config;
    bool hasEth0Config;

    uint32_t interfaceConfig; // 0xfa InterfaceConfig
    bool hasIfConfig;

    bool hasGlobalOfs;
    int modulationFreq; // in Hz; -1 if not present

    std::string _out_depth, _out_ampl,
        _out_mf, _out_it,
        _out_fc, _out_ts,
        _out_mt, _out_lt, _out_gt; ///< Input image name

    std::string camType;
    toffy::cam::CameraPtr cam;

    void setOutputsDynamic(const Frame &in, Frame& out, const boost::posix_time::ptime& start, char* data);

    void setOutputsClassic(const Frame &in, Frame& out, const boost::posix_time::ptime& start, char* data);
    void setOutputsClassicDistAmpl(const Frame &in, Frame& out, const boost::posix_time::ptime& start, char* data);
    void setOutputsClassicXYZ(const Frame &in, Frame& out, const boost::posix_time::ptime& start, char* data);
    void setOutputsClassicXYZAmpl(const Frame &in, Frame& out, const boost::posix_time::ptime& start, char* data);
    void setOutputsClassicZAmpl(const Frame &in, Frame& out, const boost::posix_time::ptime& start, char* data);
    void setOutputsClassicRawPhases(const Frame &in, Frame& out, const boost::posix_time::ptime& start, char* data);
};

}}
