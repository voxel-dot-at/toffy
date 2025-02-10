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
#include <opencv2/core/core.hpp>

/**
 * @brief
 *
 */
namespace toffy {
namespace filters {
namespace f3d {

/*class TranformUnit {
    public:
    cv::Vec3d vect;
    int op;
};*/

class Transform : public Filter
{
    std::vector<cv::Vec3d> _actions;
    std::vector<int> _operations;
    std::string _in_cloud, _out_cloud;
    static std::size_t _filter_counter;

   public:
    Transform();
    virtual ~Transform() {}
    static const std::string id_name;

    virtual bool filter(const Frame& in, Frame& out) const;

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree& pt);

    enum Operation
    {
        rotation,
        translation,
        scaling
    };
};

}  // namespace f3d
}  // namespace filters
}  // namespace toffy
