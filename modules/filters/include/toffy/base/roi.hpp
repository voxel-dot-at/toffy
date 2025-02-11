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

#include <opencv2/core.hpp>

namespace toffy {
namespace filters {

/**
 * @brief Region of interest selector and filter
 * @ingroup Filters
 *
 * For detailled information see \ref roi_page description page
 *
 */
class Roi : public Filter
{
   public:
    static const std::string id_name;  ///< Filter identifier

    Roi();
    virtual ~Roi() {}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree& pt);

    virtual bool filter(const Frame& in, Frame& out);

   private:
    std::string _in_img,  ///< Input image name
        _out_img;         ///< Output filtered image name
    double _x,            ///< Start x coordinate of the roi
        _y,               ///< Start y coordinate of the roi
        _width,           ///< Horizontal size of the roi from _x
        _height,          ///< vertical size of the roi from _y
        _inValue,   ///< Value to be check against the input data of the roi
        _outValue;  ///< Value to be set in the filtered pixels.
    bool
        _in, /**< Flag indicating if we should check and filter the inner part of the roi or the outside*/
        _filter,  ///< Flag to activate filtering by _inValue and _below flag
        _below; /**< Flag to indicate if the values over of bellow the _inValue
		well be check */
    static std::size_t _filter_counter;  ///< Internal filter counter
    cv::Rect _roi;                       ///< OpenCV rectangle defining the roi
};
}  // namespace filters
}  // namespace toffy
