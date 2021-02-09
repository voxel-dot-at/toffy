/*
   Copyright 2012-2021 Simon Vogl <svogl@voxel.at> VoXel Interaction Design - www.voxel.at

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

#include <toffy_web/filterController.hpp>
#include <toffy_web/requestAction.hpp>

namespace toffy {
namespace control {

/**
 * @brief Extended controller to handle multiples bta filter at the same time
 * @ingroup Control
 *
 * Apply the Action on all bta filters declared in the filterBanks
 */
class BtaGroupController : public FilterController {
public:

    BtaGroupController() {}

    /**
     * @brief Alternative constructor with a list of filters
     * @param vec
     */
    BtaGroupController(std::vector<toffy::Filter *> vec) : _filterList(vec) {}

    virtual ~BtaGroupController() {}

    virtual bool doAction(Action &action, std::string &log);

    static FilterController* creator() {return new BtaGroupController();}

private:
    std::vector<toffy::Filter *> _filterList;
};

} // namespace controller
} // namespace toffy
