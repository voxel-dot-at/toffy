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
 * @brief Transform depth images distances from polar to cartesian coordiantes
 * using the camera lens fov
 * @ingroup Filters
 *
 * \section ex1 Xml Configuration
 * @include polar2cart.xml
 *
 */
class Polar2Cart : public Filter {
public:

    static const std::string id_name; ///< Filter identifier
    Polar2Cart();
    virtual ~Polar2Cart() {}

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    double _fovx, _fovy;
    std::string _in_img,
	_out_img,
	_in_fovx,
	_in_fovy;
    cv::Mat _cameraMatrix;
    static std::size_t _filter_counter;
};
}}
