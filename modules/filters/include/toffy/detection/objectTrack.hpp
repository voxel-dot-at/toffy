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
#pragma once


#include <toffy/filter.hpp>
#include <fstream>

/**
 * @brief
 *
 */
namespace toffy {
namespace detection {
/**
 * @brief A try of base class for all kind of detector.
 * @ingroup Detection
 *
 * This class was thought as base class for object detector. The detection will
 * algorithms will be pretty similar and just the shape and/or characteristics
 * of the target will change from one kind the object to the other.
 * (I.e. hand/arm, leg, head, etc)
 *
 * Image filtering and noise removals will be almost the same.
 *
 * @todo This is not been used yet
 *
 */
class ObjectTrack : public Filter {
public:

    static const std::string id_name; ///< Filter identifier

    ObjectTrack();

    virtual ~ObjectTrack() {}

    //virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);
private:
    std::string _in_img,
        _in_vec, _out_vec;
    static std::size_t _filter_counter;
    int cnt;
    int numObjects;
    std::string updateScript;

    void publishCount();
};
}}
