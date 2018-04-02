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

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif


namespace toffy {
namespace filters {
/**
 * @brief Apply the lens distorsion parameters rectification to the image
 * @ingroup Filters
 *
 * \section ex1 Xml Configuration
 * @include rectify.xml
 *
 */
class Rectify : public Filter {
public:
    static const std::string id_name; ///< Filter identifier

    Rectify();
    virtual ~Rectify() {}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    std::string _in_img, ///< Name of the input image
	_out_img, ///< Name of the output image
	_in_cameraMatrix,
	_in_distCoeffs;

    cv::Mat _cameraMatrix, ///< Internal use
	_distCoeffs; ///< Internal use

    bool initialized; ///< true if cameraMatrix and distCoeffs have been set up
    cv::Mat map1, map2; ///< the maps for the remap operation.

    static std::size_t _filter_counter; ///< Internal filter counter  
};
}}
