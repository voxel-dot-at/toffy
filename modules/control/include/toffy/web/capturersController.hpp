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

#include <toffy/web/filterController.hpp>
#include <toffy/web/requestAction.hpp>

namespace toffy {
namespace control {

/**
 * @brief Extended web controller that  allows to apply actions to all defined
 * capturers in the filterBanks.
 * @ingroup Control
 */
class CapturersController : public FilterController {
public:

    CapturersController() {}

    /**
     * @brief Alternative constructor with a list of filters
     * @param vec
     */
    CapturersController(std::vector<toffy::Filter *> vec) : _filterList(vec) {}

    virtual ~CapturersController() {}

    virtual bool doAction(Action &action, std::string &log);

private:
    std::vector<toffy::Filter *> _filterList;
};

} // namespace controller
} // namespace toffy
