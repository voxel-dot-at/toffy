#pragma once

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

/**
 * @brief The filter categories libtoffy ships, and how they reach the factory.
 * @ingroup Core
 *
 * `FilterFactory::createFilter()` knows one thing: how to look a creator up.
 * Every filter libtoffy ships arrives the same way a plug-in's does -- through
 * `registerCreator()` -- and these are the functions that do it. Core calls
 * each one once, from `FilterFactory::getInstance()`.
 *
 * Before `P2-4` this was a ~120-line `if/else if` chain inside
 * `createFilter()` that `new`-ed every filter class in the product, which meant
 * core included 35 headers of modules above it (including `toffy/bta/`, the one
 * module a runner cannot build) and re-tested, in C++ preprocessor, build
 * decisions CMake had already made. Three of its branches tested macros no
 * build defines for core, so three compiled filters could not be named from a
 * config file at all -- see `tests/test_filter_registry.cpp`.
 *
 * The declarations live in one header rather than in each category's own
 * because the caller is a single function in core and the set has to be
 * complete: a category that registers nothing because nobody declared its init
 * is exactly the silent loss `P2-4`'s fence is written against.
 *
 * Names are `init<Category>Filters` rather than the older
 * `<category>::initFilters` (which `viewers` and `tracking` keep, since
 * out-of-tree callers may already call them) because the categories' classes do
 * not live in one namespace: `toffy::filters`, `toffy::filters::f3d`,
 * `toffy::detection`, `toffy::import`, `toffy::capturers`. A distinct function
 * name per category is unambiguous at the call site; a per-category
 * `initFilters` would be four different functions with the same spelling and no
 * hint which one a call refers to.
 */

#include <toffy/filterfactory.hpp>

namespace toffy {

namespace filters {

/** base filters: offset, range, cond, roi, backgroundSubs, ... */
void initBaseFilters(FilterFactory &factory);

/** the capturers that are not camera-specific (csvSource) */
void initCaptureFilters(FilterFactory &factory);

/** smoothing: average, bilateral (kalmanAverage is gated on OpenCV's tracking
 *  module -- see modules/filters/src/smoothing/init.cpp) */
void initSmoothingFilters(FilterFactory &factory);

/** detection: blobs, blobsDetector, mask, objectTrack, squareDetect */
void initDetectionFilters(FilterFactory &factory);

/** import: dataImporter, importYaml */
void initImportFilters(FilterFactory &factory);

/** the 3d/PCL filters. Defined only when PCL is enabled -- the object library
 *  (`toffy_3d`) does not exist otherwise -- so callers must guard the call with
 *  `#if defined(PCL_FOUND)`, which is the same macro the build sets. */
void init3dFilters(FilterFactory &factory);

}  // namespace filters

// The two categories that registered themselves before P2-4; unchanged, and
// declared here so that core needs one include rather than a forward
// declaration block of its own (which is what it used to have).
namespace tracking {
void initFilters(FilterFactory &factory);
}  // namespace tracking

namespace viewers {
void initFilters(FilterFactory &factory);
}  // namespace viewers

}  // namespace toffy
