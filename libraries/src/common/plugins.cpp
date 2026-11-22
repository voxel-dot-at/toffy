/*
   Copyright 2018 Simon Vogl <svogl@voxel.at>

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

// The one plug-in loader. P2-10.
//
// POSIX only: toffy has no Windows plugin loader (P2-2,
// cleanup/findings/build.md). The LoadLibrary/GetProcAddress half of the old
// three copies was never compiled, because MSVC is not defined for the library
// in any configuration.

#include "toffy/common/plugins.hpp"

#include <dlfcn.h>

#include <ostream>

#include <toffy/logging.hpp>

namespace toffy {
namespace commons {
namespace plugins {

namespace {

// Process-lifetime on purpose. See loadedHandles() in the header for why the
// handles are never closed; a function-local static is used rather than a member
// of some object so that the owner is not whichever class happens to be around.
std::vector<void *> &handles()
{
    static std::vector<void *> h;
    return h;
}

}  // namespace

void *loadOne(const std::string &lib)
{
    if (lib.empty()) {
        // <plugins></plugins> is present-but-empty in testbox.xml; an entry with
        // no text is the same case one level down. dlopen("") is not a thing
        // worth asking for.
        toffy::log::debug() << "empty plug-in entry, skipped";
        return nullptr;
    }

    toffy::log::debug() << "Loading plug-in from: " << lib;
    void *libHandle = dlopen(lib.c_str(), RTLD_LAZY);
    if (!libHandle) {
        toffy::log::warning()
            << "Could not load library: " << lib << std::endl << dlerror();
        dlerror();  // reset
        return nullptr;
    }

    init_t init = reinterpret_cast<init_t>(dlsym(libHandle, "init"));
    const char *dlsym_error = dlerror();
    if (dlsym_error) {
        toffy::log::warning() << "Could not load plug-in filters from: " << lib
                             << std::endl
                             << dlsym_error;
        // Nothing was registered from this library, so unloading it is safe and
        // correct. None of the three old copies did this: they leaked the handle
        // on exactly this path.
        dlclose(libHandle);
        dlerror();  // reset
        return nullptr;
    }

    toffy::log::info() << "Loaded plug-in filters from: " << lib;
    init(FilterFactory::getInstance());
    handles().push_back(libHandle);
    return libHandle;
}

std::vector<void *> loadFromConfig(const boost::property_tree::ptree &pt,
                                   const std::string &node)
{
    std::vector<void *> opened;

    // get_child_optional, not get_child: the old Controller and FilterBank
    // versions used get_child(), which throws ptree_bad_path when the node is
    // absent. A config with no <plugins> is not an error -- wbox.xml has none --
    // and the Player version already treated it as one it could live with.
    const boost::optional<const boost::property_tree::ptree &> child =
        pt.get_child_optional(node);
    if (!child) {
        toffy::log::debug() << "no <" << node
                           << "> node in config; nothing to load";
        return opened;
    }

    for (const boost::property_tree::ptree::value_type &v : *child) {
        void *libHandle = loadOne(v.second.data());
        if (libHandle) opened.push_back(libHandle);
    }

    return opened;
}

const std::vector<void *> &loadedHandles() { return handles(); }

}  // namespace plugins
}  // namespace commons
}  // namespace toffy
