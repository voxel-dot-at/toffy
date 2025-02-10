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

#include "toffy/filter.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief
 *
 */
namespace toffy {
namespace filters {
namespace f3d {

class GroundProjection : public Filter
{
   public:
    static const std::string id_name;
    GroundProjection();
    virtual ~GroundProjection() {}

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree& pt);

    virtual bool filter(const Frame& in, Frame& out);

   private:
    std::string _in_cloud, _in_img, _out_img;
    float _max_y, _max_x;
    int _scale;
    double _fovx, _fovy, _dis;
    bool _projBack;
    cv::Mat _cameraMatrix;
    static std::size_t _filter_counter;
};

}  // namespace f3d
}  // namespace filters
}  // namespace toffy
