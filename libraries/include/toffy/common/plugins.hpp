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

#include <string>
#include <vector>

#include <boost/property_tree/ptree.hpp>

#include "toffy/filterfactory.hpp"

namespace toffy {
namespace commons {
namespace plugins {
extern "C" {
void init(FilterFactory *ff);
}
typedef void (*init_t)(FilterFactory *);

/**
 * @brief Load every plug-in named under `node` in `pt`. P2-10.
 *
 * This is the one implementation. There used to be three -- Player,
 * Controller and FilterBank each had a `loadPlugins` that walked a property tree
 * and called `dlopen` -- and they disagreed in ways that mattered:
 *
 * | loader | node looked up | missing node | dlopen handle |
 * |---|---|---|---|
 * | `Player::loadPlugins` | `toffy.plugins` | caught, warned | stored on `Controller::_loads` |
 * | `Controller::loadPlugins` | `toffy.plugins` | **threw `ptree_bad_path`** | stored on `_loads` |
 * | `FilterBank::loadPlugins` | `plugins` | **threw `ptree_bad_path`** | **discarded** |
 *
 * The throw matters because a config with no `<plugins>` node is ordinary:
 * `becom-knapp-wurthbox`'s `wbox.xml` has none. Only `Player::loadConfig()` is on
 * the live path, and it is the one that catches -- which is why production works
 * and the other two have never shown up as a failure.
 *
 * A missing node is not an error here: it means there is nothing to load.
 *
 * @param pt the parsed config
 * @param node path to the plugins node, defaulting to the one every config in
 *        this tree and both consumers actually uses (`toffy.plugins`)
 * @return the handles newly opened by this call, in order
 */
std::vector<void *> loadFromConfig(const boost::property_tree::ptree &pt,
                                   const std::string &node = "toffy.plugins");

/**
 * @brief Load one plug-in by path, run its init(), and record the handle.
 *
 * The single `dlopen` path. `Controller::loadPlugin()` is public API and takes a
 * bare path rather than a config node, so it needs this; before P2-10 it carried
 * its own copy of the dlopen/dlsym/init sequence, which is where the three
 * versions could drift apart in the first place.
 *
 * @return the handle, or nullptr if the library or its `init` could not be found
 */
void *loadOne(const std::string &lib);

/**
 * @brief Every plug-in handle this process has successfully opened.
 *
 * Single owner, which is the point: the handles used to be split between
 * `Controller::_loads` and a `FilterBank` local that was thrown away.
 *
 * They are never `dlclose`d, and that is deliberate rather than an oversight --
 * see the note in plugins.cpp. A plug-in's `init()` registers creators into the
 * process-wide `FilterFactory`, and those creators are function pointers into the
 * `.so`; unloading it while the registry still holds them makes the next
 * `createFilter()` jump into unmapped memory. Closing them safely needs the
 * registry to give creators back first, which it cannot do today.
 */
const std::vector<void *> &loadedHandles();

}}}
