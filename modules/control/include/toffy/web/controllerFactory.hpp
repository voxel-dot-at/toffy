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

//#include <toffy/toffy_export.h>
#include <toffy/web/filterController.hpp>

namespace toffy {
namespace control {

/**
   * type signature for functions returning new a filter of an (unknown) type.
   */
typedef control::FilterController* (*CreateFilterControllerFn)(void);

/**
 * @brief a copy of the FilterFactory for registing web ui controllers from
 * outside filter.
 * @ingroup Control
 *
 * @todo Right now it is not used anywhere. Its the best way for expand
 * control?, where to put it, handle_request of inside the base controller?,
 * how it should looks like?.
 */
class TOFFY_EXPORT ControllerFactory {
    static ControllerFactory *_uniqueFactory;
    static boost::container::flat_map<std::string, FilterController *> _controllers;
    ControllerFactory();
public:
    static ControllerFactory * getInstance();

    virtual ~ControllerFactory();

    control::FilterController *createController(Filter *f);

    int deleteController(const std::string& name);

    //int updateFilter(std::string name, const cv::FileNode &fn) const;

    control::FilterController * getController(Filter *f);

    bool findController(std::string name) const {
	return _controllers.find(name) == _controllers.end() ? false : true;
    }

    int getControllersByType(const std::string& type,
			     std::vector<control::FilterController *> &vec);


    /** register a filter creation function with a name.
	 * @param name the name of the filter
	 * @param fn function pointer to the instantiation code
	 */
    static void registerCreator( std::string name, CreateFilterControllerFn fn);

    // unregister a filter creation function
    static void unregisterCreator(std::string name);

    void setCtrl(toffy::Controller *c) {_c = c;}

private:
    static boost::container::flat_map<std::string, CreateFilterControllerFn> _creators;
    toffy::Controller *_c;
};

}}
