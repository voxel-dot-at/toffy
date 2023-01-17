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
#include <toffy/bta/BtaWrapper.hpp>

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

    //TODO MOVE TO OPENCV
    //virtual void setCamera2Wcs(toffy::Frame& out, std::string name);

private:
    // frame size data
    int distsSize;
    int width, height;
    bool dynOutputs; // dynamically map output depending on channels present

    float globalOfs;
    bool hasGlobalOfs;
    int modulationFreq; // in Hz; -1 if not present

    std::string _out_depth, _out_ampl,
        _out_mf, _out_it,
        _out_fc, _out_ts,
        _out_mt, _out_lt, _out_gt; ///< Input image name


    void setOutputsClassic(const Frame &in, Frame& out, boost::posix_time::ptime start, char* data);
    void setOutputsDynamic(const Frame &in, Frame& out, boost::posix_time::ptime start, char* data);
};

}}
