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

#include <string>

#include "toffy/filterbank.hpp"

namespace toffy {
namespace filters {
/**
 * @brief A conditional filter - run the encapulating filterbank if a condition
 * is met.
 * @ingroup Filters
 *
 */
class Cond : public FilterBank
{
   public:
    Cond();
    virtual ~Cond() {}

    virtual boost::property_tree::ptree getConfig() const;

    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

    void enable() { enabled = true; }
    void disable() { enabled = false; }

    // derived from FilterBank:
    virtual int loadConfig(const boost::property_tree::ptree& pt, const std::string& confFile = "");
    
   private:
    bool enabled;  ///< switch if the contained filters shall be run or not
    std::string opt_file;  ///< the filename of the sub-filterbank to load
};

}  // namespace filters
}  // namespace toffy
