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
 * @brief Compute a focus measure for the image sharpness
 * @ingroup Filters
 *
 * \section ex1 Xml Configuration
 * @include focus.xml
 *
 */
class Focus : public Filter
{
   public:
    static const std::string id_name;  ///< Filter identifier

    Focus();
    virtual ~Focus() {}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree& pt);

    virtual bool filter(const Frame& in, Frame& out);

   private:
    std::string in_img;     ///< Name of the input image
    std::string out_focus;  ///< Name of the key for the focus variable double
};
}  // namespace filters
}  // namespace toffy
