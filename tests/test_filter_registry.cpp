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
//     below. Pinned as it behaves, not as anyone would choose -- and pinned only
//     where the class is actually compiled, because a second machine showed the
//     unconditional version passing on a build where one of the three classes
//     was not in the library at all.
//
// What measuring the target machine added, beyond the above: `cvTracker` was
// missing from the expected set entirely. It has been a registered, name-
// creatable type the whole time (`tracking/init.cpp` registers it under
// `#if OPENCV_TRACKING`, a macro that directory does see) with nothing here
// requiring it to stay that way. The fence only protects names it names.
//
// The set was measured, not transcribed: every name below was passed to
// `createFilter()` against a Release/PCL-on/BTA-on build of the pre-move tree
// and its verdict recorded. The three that came back "Unknown filter" are the
// finding -- their branches test macros that no build defines for the
// translation unit holding the chain (`WITH_VISUALIZATION` is defined nowhere
// at all; `OPENCV_TRACKING` and `PCL_VISUALIZATION` are defined only inside
// `modules/filters/src/{smoothing,viewers}/`, and the chain lives in
// `modules/core`). So three filters that are compiled into `libtoffy.so` could
// not be instantiated by name: a macro the build never sets, gating something
// the build does compile -- the bug class the deleted web-control hook was
// (counter 26), in a new place. They are left unreachable by the move
// (deleting a branch that can
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
#include <iostream>
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

#if defined(OPENCV_TRACKING)
    // Found by measuring the target machine rather than reading the tree. This
    // one was missing from the fence entirely: `tracking/init.cpp` registers
    // `CVTracker::id_name` under `#if OPENCV_TRACKING`, and the macro *is* set
    // for that directory, so `cvTracker` has been a working, registered, name-
    // creatable filter type all along -- measured `hasCreator=1 creatable=1` on
    // a PCL-on/BTA-on host build -- with nothing in this file asserting it
    // should stay that way. A type the fence does not name is a type the fence
    // cannot protect.
    t.push_back("cvTracker");
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

// `hasCreator()` reads a registry that `getInstance()` populates lazily, so
// asking it before anything has asked for an instance answers "no" for every
// name in the product. Measured: a probe that called `hasCreator("cvTracker")`
// before its first `getInstance()` reported `hasCreator=0 creatable=1`. Every
// helper here warms the factory first so the answer means what it looks like it
// means, independent of which test ran before it.
void warmRegistry()
{
    FilterFactory::getInstance();
}

std::vector<std::string> missingFromRegistry()
{
    warmRegistry();
    std::vector<std::string> missing;
    for (const auto &name : expectedTypes()) {
        if (!FilterFactory::hasCreator(name)) missing.push_back(name);
    }
    return missing;
}

// Which of the deliberately-unreachable names have their *class* compiled in
// this configuration. The distinction is the point: asserting that a name does
// not resolve proves nothing about the registry when the class behind it was
// never built. That is the difference between a decision ("we have not
// registered this yet, on purpose") and an accident ("this build does not
// contain the filter at all"), and the old test certified both identically.
struct UnreachableName
{
    const char *name;
    bool classCompiled;
    const char *why;
};

std::vector<UnreachableName> unreachableNames()
{
    return {
        {"reprojectopencv", true, "reprojectopencv.cpp is in the source list "
                                  "unconditionally"},
        {"reprojectpcl",
#if defined(PCL_FOUND)
         true, "built when PCL_FOUND"},
#else
         false, "not built: PCL is off"},
#endif
        {"kalmanAverage",
#if defined(OPENCV_TRACKING)
         true, "built when OpenCV's `tracking` module is found"},
#else
         false, "not built: OpenCV `tracking` module absent"},
#endif
    };
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
// the chain that no build could reach, because the guard tested a macro that is
// not defined for modules/core. Where the class *is* compiled, the filter exists
// inside libtoffy.so and still cannot be named from a config file -- which is the
// defect, and the reason these names are pinned rather than fixed here.
//
// This test exists so that the next person to register one of these names is told
// the set changed and why, instead of finding out from a config that suddenly
// works.
//
// It used to assert non-creatability for all three unconditionally, and that was
// wrong in a way only a second machine exposed. On the 2026-11-20 target build
// (Ubuntu 18.04, OpenCV 4.1 without the contrib `tracking` module) `kalmanaverage.cpp`
// is not compiled at all -- `nm` on the resulting libtoffy: zero matching symbols
// -- so the assertion passed there with nothing behind it. A test that cannot
// fail in a configuration is not pinning anything in that configuration; it is
// just quiet. Hence: assert only where the class exists, and say out loud where
// it does not.
TEST(FilterRegistry, ThreeChainNamesAreUnreachableToday)
{
    warmRegistry();

    int asserted = 0;
    for (const auto &u : unreachableNames()) {
        if (!u.classCompiled) {
            // Non-creatability here is a property of the build, not of the
            // registry. Recorded rather than counted as a pass.
            std::cout << "  [skipped] '" << u.name << "': " << u.why << "\n";
            RecordProperty("skipped_unreachable", u.name);
            continue;
        }
        ++asserted;
        EXPECT_FALSE(FilterFactory::hasCreator(u.name))
            << "'" << u.name << "' is now a registered creator while its class "
               "is also compiled -- it became resolvable. Update this test and "
               "cleanup/counters.md: the resolvable set has moved.";
        EXPECT_EQ(nullptr, FilterFactory::getInstance()->createFilter(u.name))
            << "'" << u.name << "' became creatable. Update this test and "
               "cleanup/counters.md: the resolvable set has moved.";
    }

    // `reprojectopencv` is built unconditionally, so this can only fire if that
    // stops being true -- at which point the test below has silently emptied.
    EXPECT_GT(asserted, 0)
        << "no unreachable name's class is compiled in this configuration, so "
           "this test asserted nothing. That is a build-configuration change, "
           "not a pass.";
    RecordProperty("unreachable_asserted", asserted);
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
