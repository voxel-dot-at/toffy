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
#  include <opencv2/imgproc.hpp>
#else
#  include <opencv2/core/core.hpp>
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/contrib/contrib.hpp>
#endif

namespace toffy {
/**
 * @brief Colorizes a single-channel Mat (preferrably depth data) 
 * @ingroup Viewers
 *
 * Takes one single image from the frame and converts it to a Mat in the output frame
 *
 * It allows also to scale up or down the image keeping the proportions. 
 *
 * \section ex1 XML Configuration
 * <h4>Inputs</h4>
 * <ul>
 * <li> img - the input image to convert 
 * </ul
 * <h4>Ouputs</h4>
 * <ul>
 * <li> img - the output image (RGB)
 * </ul
 * <h4>Options</h4>
 * <ul>
 * <li> min - (float) min value for the colormap 
 * <li> max - (float) max value for the colormap
 * <li> colormap - one of jet / hot/ hsv / rainbow (defaults to jet)
 * <li> scale - scale factor 
 * </ul>
 *
 */
class Colorize : public Filter	{
public:
    Colorize();
    virtual ~Colorize();

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

    static const std::string id_name; ///< Filter identifier
private:
    double scale, ///< Scale the image keeping the proportions
	max, ///< Max value for converting to grayscaled
	min; ///< Min value for converting to grayscaled
    std::string in_img, out_img, colormap;
#if OCV_VERSION_MAJOR >= 3
    cv::ColormapTypes colormap_value;
#else
    int colormap_value;
#endif
    bool _gray; ///< Flag for converting the image to grayscaled.

    static std::size_t filter_counter; 

    cv::Mat gray, col, scaled, bounds; 
};
}
