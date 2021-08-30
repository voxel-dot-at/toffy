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
/**
 * @brief Display a OpenCV Mat image int a windows viewer
 * @ingroup Viewers
 *
 * Takes one single image form the frame and shows is using ImageView from
 * OpenCV.
 *
 * It allows also to scale up or down the image keeping the proportions. Also
 * the image can be convert and display in grayscale
 *
 * \section ex1 Xml Configuration
 * @include imageview.xml
 *
 */
class ImageView : public Filter	{
public:

    static const std::string id_name; ///< Filter identifier
    ImageView();
    virtual ~ImageView();

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    double _scale, ///< Scaled the image keeping the proportions
	_max, ///< Max value for converting to grayscaled
	_min; ///< Min value for converting to grayscaled
    std::string _in_img;
    bool _gray; ///< Flag for converting the image to grayscaled.
    bool _enabled; ///< if enabled, show the image, otherwise skip it
    int _waitKey; ///< timeout for waitKey, set via options. -1 if disabled
    static std::size_t _filter_counter; ///< Internal filter counter
    cv::Mat show;
};
}
