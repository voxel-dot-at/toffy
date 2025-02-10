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

#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief
 *
 */
namespace toffy {
namespace filters {
namespace smoothing {
/** perform bilateral filtering of the (depth) channel.
	 */
class Bilateral : public Filter
{
    static std::size_t _filter_counter;

   public:
    Bilateral();

    virtual ~Bilateral() {}

    static const std::string id_name;

    virtual bool filter(const Frame& in, Frame& out);

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree& pt);

   private:
    std::string _in_img, _out_img;
    double d, sigmaColor, sigmaSpace;
    cv::Mat _dst;
};
}  // namespace smoothing
}  // namespace filters
}  // namespace toffy
