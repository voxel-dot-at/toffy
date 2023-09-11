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
#include <toffy/cam/cameraParams.hpp>

namespace toffy {
namespace detection {
/**
  *
@brief Square detector:
@ingroup Detection

For detailed information see \ref handDetect_page description page
 *
 */
class SquareDetect: public Filter
{
public:
    static const std::string id_name;
    SquareDetect();
    virtual ~SquareDetect();

    /** call the filter functions in the filter bank
     */
    bool filter(const Frame& in, Frame& out);

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

private:
    std::string in_depth, ///< depth input mat
	in_ampl, ///< ampl input mat
	in_blobs, ///< ampl detected object vector
	out_detect; ///<  detected objects visu
    cv::Mat _cameraMatrix;

    static std::size_t _filter_counter;
    toffy::cam::CameraPtr cam;
};

}}
