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

// P2-4 fence: every built-in filter type is a *registration*, not a branch.
//
// Why this file exists before the change rather than after it. `P2-4` deletes a
// 35-branch `if/else if` chain in `FilterFactory::createFilter()` and moves each
// branch into a `registerCreator()` call in its own module. That is a mechanical
// move of ~35 lines across 8 files, and the only failure mode that matters is
// the one a compiler cannot see: a type name that quietly stops resolving. A
// config file naming it fails at run time, in the field, on the filter nobody
// exercised in CI -- and `boxes` and `toffy-oatpp` both configure filters by
// name out of XML, so "it built" is not evidence at all here.
//
// So the fence is the set, not the code:
//
//   * `EveryExpectedTypeIsCreatable` -- the resolvable set, green before the
//     move and green after it. This is the characterisation half: it says the
//     move did not lose or gain a type.
//   * `EveryExpectedTypeIsARegisteredCreator` -- the half that is RED on the
//     parent commit (32 of the 33 names below are reachable only through the
//     chain, so `hasCreator()` is false for them) and green after it. Without
//     this test the move could be "done" with the chain still in place and
//     every other assertion here would still pass.
//   * `ThreeChainNamesAreUnreachableToday` -- found by writing this file, see
//     below. Pinned as it behaves, not as anyone would choose.
//
// The set was measured, not transcribed: every name below was passed to
// `createFilter()` against a Release/PCL-on/BTA-on build of the pre-move tree
// and its verdict recorded. The three that came back "Unknown filter" are the
// finding -- their branches test macros that no build defines for the
// translation unit holding the chain (`WITH_VISUALIZATION` is defined nowhere
// at all; `OPENCV_TRACKING` and `PCL_VISUALIZATION` are defined only inside
// `modules/filters/src/{smoothing,viewers}/`, and the chain lives in
// `modules/core`). So three filters that are compiled into `libtoffy.so` could
// not be instantiated by name: the `WITH_CONTROL` bug class (counter 26) in a
// new place. They are left unreachable by the move (deleting a branch that can
// never be taken must not silently become a new feature) and fixed in their own
// commit, where the guard that gates them is one the build actually sets.
//
// Nothing in here instantiates a viewer: `CloudViewOpenCv`'s constructor opens
// a VTK render window and dies with "bad X server connection" on a headless
// runner, which is a fact about the class, not about the factory. Those names
// are checked with `hasCreator()` only.

#include <gtest/gtest.h>

#include <toffy/filterfactory.hpp>

#include <algorithm>
#include <string>
#include <vector>

using toffy::FilterFactory;

namespace {

// The types `createFilter()` must produce in this configuration. Guarded the
// way the build guards them, so the same file is honest in all four
// PCL x BTA cells: PCL_FOUND and HAS_BTA are set at the top level and reach
// every target, which is exactly why the PCL types below are reachable through
// the chain and the PCL_VISUALIZATION ones are not.
std::vector<std::string> expectedTypes()
{
    std::vector<std::string> t = {
        // core's own
        "filterBank",
        "parallelFilter",
        // base
        "offset",
        "distAmpl",
        "offsetCorr",
        "polar2cart",
        "rectify",
        "amplitudeRange",
        "range",
        "nop",
        "cond",
        "focus",
        "roi",
        "backgroundSubs",
        // capture
        "csvSource",
        // smoothing (the two that are not conditional)
        "bilateral",
        "average",
        // detection
        "objectTrack",
        "blobs",
        "blobsDetector",
        "mask",
        // import
        "dataImporter",
        "importYaml",
    };

#if defined(PCL_FOUND)
    // 3d + the two viewers/detection types the chain gates on PCL. Note
    // `exportYaml` and `squareDetect`: their classes are built unconditionally
    // and use no pcl at all (exportYaml's pcl includes are commented out), so
    // the PCL guard on them is vestigial. Preserved by the move, filed as a
    // follow-up -- see the note in the header.
    const char *pcl_types[] = {"transform",   "merge",     "xyz2pcl",
                               "split",       "muxMerge",  "sampleConsensus",
                               "exportcloud", "exportYaml", "groundprojection",
                               "squareDetect"};
    t.insert(t.end(), std::begin(pcl_types), std::end(pcl_types));
#endif

    return t;
}

// Names the modules register for themselves (they were never in the chain).
// Listed so the move has to leave them alone; checked without instantiating.
const std::vector<std::string> &registeredByModules()
{
    static const std::vector<std::string> n = {"colorize", "exportcsv",
                                               "imageview", "videoout",
                                               "tracker"};
    return n;
}

std::vector<std::string> missingFromRegistry()
{
    std::vector<std::string> missing;
    for (const auto &name : expectedTypes()) {
        if (!FilterFactory::hasCreator(name)) missing.push_back(name);
    }
    return missing;
}

std::vector<std::string> notCreatable()
{
    std::vector<std::string> failed;
    for (const auto &name : expectedTypes()) {
        if (!FilterFactory::getInstance()->createFilter(name)) {
            failed.push_back(name);
        }
    }
    return failed;
}

std::string join(const std::vector<std::string> &v)
{
    std::string s;
    for (const auto &e : v) { s += "\n      "; s += e; }
    return s;
}

}  // namespace

// A duplicated entry would make the two set tests weaker than they look (the
// second copy of a dropped name cannot fail if the first one passes), so the
// list checks itself first.
TEST(FilterRegistry, ExpectedSetHasNoDuplicates)
{
    const auto t = expectedTypes();
    std::vector<std::string> sorted(t);
    std::sort(sorted.begin(), sorted.end());
    auto dup = std::adjacent_find(sorted.begin(), sorted.end());
    EXPECT_EQ(sorted.end(), dup)
        << "'" << *dup << "' appears twice in the expected type list";
}

// Characterisation half: every name in the list resolves. Green before the
// move, green after it; anything red here is a type the move lost.
TEST(FilterRegistry, EveryExpectedTypeIsCreatable)
{
    const auto failed = notCreatable();
    EXPECT_TRUE(failed.empty())
        << "createFilter() cannot produce " << failed.size()
        << " type(s) it used to:\n      " << join(failed);
}

// The fence. `createFilter()` may keep a chain and still pass every other test
// in this file; it may not pass this one. Every built-in type has to be
// reachable the same way a plug-in's type is: through the registry.
//
// RED on the parent commit: 32 of the 33 names are reachable only through the
// chain, so `hasCreator()` is false for them (the one exception is
// `exportcloud`, which `viewers/init.cpp` already registers and the chain
// duplicated).
TEST(FilterRegistry, EveryExpectedTypeIsARegisteredCreator)
{
    const auto missing = missingFromRegistry();
    EXPECT_TRUE(missing.empty())
        << missing.size()
        << " built-in type(s) are hard-coded in createFilter() instead of being "
           "registered creators:\n      "
        << join(missing);
}

// The module-registered names are part of the factory's surface too. Checked
// with hasCreator() rather than by construction: these are the viewers, and
// constructing them needs a display.
TEST(FilterRegistry, ModuleRegisteredTypesSurviveTheMove)
{
    for (const auto &name : registeredByModules()) {
        EXPECT_TRUE(FilterFactory::hasCreator(name))
            << "'" << name << "' is no longer registered";
    }
}

// Pinned as it behaves, not as anyone would choose: three names had branches in
// the chain that no build can reach, because the guard tests a macro that is
// not defined for modules/core. The classes are compiled into libtoffy.so --
// so the filters exist in the library and cannot be named from a config file.
//
// This test exists so that the next person to define one of those macros, or to
// register one of these names, is told that the set changed and why, instead of
// finding out from a config that suddenly works.
TEST(FilterRegistry, ThreeChainNamesAreUnreachableToday)
{
    for (const auto &name : {"reprojectopencv", "kalmanAverage", "reprojectpcl"}) {
        EXPECT_EQ(nullptr, FilterFactory::getInstance()->createFilter(name))
            << "'" << name << "' became creatable -- its guard now reaches "
               "modules/core, or it was registered. Update this test and "
               "cleanup/counters.md: the resolvable set has moved.";
    }
}

// The registry is insert-does-not-overwrite, and the built-ins are registered
// during the first getInstance(). So a consumer that registers a name the
// factory already has keeps the factory's version -- which is what the chain
// did (the chain was tried first), and the one behaviour the move is not
// allowed to change under someone's feet.
TEST(FilterRegistry, RegisteringABuiltInNameLaterDoesNotReplaceIt)
{
    const std::string name = "nop";
    ASSERT_NE(nullptr, FilterFactory::getInstance()->createFilter(name));

    FilterFactory::registerCreator(
        name, []() -> toffy::Filter * { return nullptr; });

    auto f = FilterFactory::getInstance()->createFilter(name);
    EXPECT_NE(nullptr, f)
        << "'" << name << "' was overridden by a later registerCreator()";
}
