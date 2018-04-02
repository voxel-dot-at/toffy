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

namespace toffy {
namespace filters {

/**
 * \brief Filters amplitude data and it corresponding pixels in the depth image
 * \ingroup Filters
 *
 * For detailled information see \ref amplitudeRange_page description page
 *
 */
class AmplitudeRange : public Filter {
public:
    static const std::string id_name; ///< Filter identifier

    AmplitudeRange();
    virtual ~AmplitudeRange() {}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);
private:
    std::string _in_ampl,
	_in_depth,
	_out_ampl,
	_out_depth;
    double _minAmpl,
	_maxAmpl;
    static std::size_t _filter_counter;
};
}
}
