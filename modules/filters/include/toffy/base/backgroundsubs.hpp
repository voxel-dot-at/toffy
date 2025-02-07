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

namespace toffy {
namespace filters {

/**
 * @brief Creates a background mask from an averaged input view and filters
 * the input images with it.
 *
 * @ingroup Filters
 *
 * For detailed information see \ref backgroundSubs_page description page
 *
 */
class BackgroundSubs: public Filter
{
public:
    static const std::string id_name; ///< Filter identifier

    BackgroundSubs();
    virtual ~BackgroundSubs();

    virtual bool filter(const toffy::Frame& in, toffy::Frame& out);

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

private:
    std::string in_img, ///< Input image
	out_img, ///< output image after filter
	_in_mask; ///< Input mask file, @todo manually set, move to config
    bool _creation, ///< flag to create bg mask (auto-set to false when done)
	_filterSpotNoise, ///< enable noise filter
	_median; ///< enable median filter
    int _neighbours, ///< minimum number of neighbours for noise @todo manually set, move to config
	_ite; ///< #frames to capture @todo manually set, move to config
    double _offset; ///< offset: added to the mask to guard against distance noise.
    FilterPtr _avg; /**< Filter in filter we use averaging
	in the input data*/
    matPtr avgdImg;
    static std::size_t _filter_counter;

    /**
     * @brief loadAvgData
     * @param inImg
     * @return True on success, False if failed
     *
     * Loads a mask image from a file and averages it with the input mask data
     * if not empty.
     *
     */
    bool loadAvgData(matPtr inImg);
};

}}
