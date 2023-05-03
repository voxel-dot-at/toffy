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

#include <toffy/filter.hpp>
#include <toffy/controller.hpp>
#include <toffy/toffy_export.h>

#include <toffy/web/requestAction.hpp>
//#include <toffy/web/requestController.hpp>

namespace toffy {
namespace control {

/**
 * @brief Base class for expanding filter web ui controllers.
 * @ingroup Control
 *
 * @todo As said in FilterFactory, to be changed.
 */
class TOFFY_EXPORT FilterController {
public:

    FilterController() {}

    /**
     * @brief Alternative constructor with Filter
     * @param f
     */
    FilterController(toffy::Filter *f) {_f = f;}

    virtual ~FilterController() {}

    /**
     * @brief Setter for the filter to be handled
     * @param f
     */
    void setFilter(toffy::Filter *f) {_f = f;}

    virtual toffy::Controller * getBaseController(){return _base;}
    virtual void setBaseController(toffy::Controller *c) {_base = c;}

    virtual toffy::Filter * filter() {return _f;}

    /**
     * @brief Executes on filter the given action.
     * @param action
     * @param log
     * @return
     */
    virtual bool doAction(Action &action, std::string &log);

    virtual bool getMetaData(Action &action, std::string &log);

private:
    toffy::Filter *_f;
    toffy::Controller *_base;
};

} // namespace controller
} // namespace toffy
