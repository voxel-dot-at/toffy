/*
   Copyright 2026 Simon Vogl <svogl@voxel.at>

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

// The capturers that are not camera-specific (P2-4). `bta` registers itself --
// see modules/bta/src/initPlugin.cpp -- and lives in a module a CI runner cannot
// build; this one is generic CSV playback and is always built, so it always
// registers.

#include <toffy/filter_init.hpp>

#include "toffy/io/csv_source.hpp"

namespace toffy {
namespace filters {

void initCaptureFilters(FilterFactory &factory)
{
    factory.registerCreator("csvSource", []() -> Filter * {
        return new capturers::CSVSource();
    });
}

}  // namespace filters
}  // namespace toffy
