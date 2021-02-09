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
#include <toffy_web/action.hpp>
//#include <toffy/bta.hpp>


namespace toffy {
namespace control {

/**
 * @brief Web controller class for handle the specific operations on the
 * bta filter.
 */
class BtaController : public FilterController {
public:

    BtaController() {}

    /**
     * @brief Constructor for filling the base class
     * @param f Filter in toffy to be handled
     */
    BtaController(toffy::Filter *f) : FilterController(f) {}

    virtual ~BtaController() {}
    virtual bool doAction(Action &action, std::string &log);

    static FilterController* creator() {return new BtaController();}
private:
};

} // namespace controller
} // namespace toffy
