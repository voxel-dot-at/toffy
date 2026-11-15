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

// Unit tests for toffy::Controller construction and teardown.

#include <gtest/gtest.h>

#include <stdexcept>

#include <toffy/base/nop.hpp>
#include <toffy/controller.hpp>
#include <toffy/filterfactory.hpp>

using toffy::Controller;

// Regression tests for cleanup/plan/p0-correctness.md P0-9 / finding C3.
//
// Controller's constructor did `baseFilterBank = createFilter("filterBank", ...)`
// and then immediately called ->bank(NULL) on the result with no null check, and
// ~Controller called ->id() the same way.
//
// Honest scope note: the null path is NOT reachable from the public API today.
// FilterFactory::createFilter() resolves "filterBank" through a hard-coded
// branch that cannot fail short of bad_alloc, so there is no way to write a test
// that fails on the parent commit for this specific fault (DOD 1.3 cannot be met
// here). The guards are therefore defensive. What these tests do pin down is that
// the guards do not fire spuriously and that the lifecycle works -- including
// across two Controllers in one process, which is where ~Controller used to have
// a process-global side effect on the creator registry (see below).
TEST(ControllerLifecycle, ConstructAndDestroy)
{
    ASSERT_NO_THROW({
        Controller c;
        EXPECT_NE(nullptr, c.baseFilterBank);
    });
}

// Two Controllers in one process, sequentially. This is the test that caught
// P2-4's one non-mechanical consequence: ~Controller used to call
// FilterFactory::clearCreators(), wiping the process-wide creator registry, and
// it survived that only because the built-in types resolved through
// createFilter()'s hard-coded chain. With the chain gone the second Controller
// could not create its base bank --
//
//     error: Unknown filter: filterBank
//     what(): Controller: FilterFactory could not create the base FilterBank
//
// -- which is what this test fails with on the intermediate commit, and why the
// clearCreators() call came out of ~Controller rather than the built-ins being
// re-registered behind it. A Controller does not own the registry.
TEST(ControllerLifecycle, SequentialControllersStillWork)
{
    ASSERT_NO_THROW({ Controller first; });
    ASSERT_NO_THROW({
        Controller second;
        EXPECT_NE(nullptr, second.baseFilterBank);
    });
}

// Finding C3, the half of it that outlived P2-5: ~Controller called
// FilterFactory::clearCreators(), which empties a *process-wide* registry from
// the destructor of one object. A consumer registers its own filters once at
// startup -- `boxes` and `toffy-oatpp` both do it in `init_toffy()` -- and names
// them from a config file afterwards, so destroying a Player (and with it a
// Controller) in a long-running process silently lost every one of them: the
// next config load reported "Unknown filter" for a type the binary had.
//
// Fails on the parent commit with
//
//     Value of: FilterFactory::hasCreator("test.survivesControllerTeardown")
//       Actual: false   Expected: true
//
// and passes with the call removed. Nothing else in the tree calls
// clearCreators(); the method stays public because removing it is an API break.
TEST(ControllerLifecycle, DestroyingAControllerKeepsRegisteredCreators)
{
    const std::string name = "test.survivesControllerTeardown";
    toffy::FilterFactory::registerCreator(
        name, []() -> toffy::Filter* { return new toffy::filters::Nop(); });
    ASSERT_TRUE(toffy::FilterFactory::hasCreator(name));

    {
        Controller c;
        EXPECT_NE(nullptr, c.baseFilterBank);
    }

    EXPECT_TRUE(toffy::FilterFactory::hasCreator(name))
        << "~Controller wiped the process-wide creator registry: '" << name
        << "' is gone, and with it every plug-in and Player::loadFilter() "
           "creator in the process";
    EXPECT_NE(nullptr, toffy::FilterFactory::getInstance()->createFilter(name))
        << "the name is still registered but no longer resolves";
}

// The base bank must not report itself as living in another bank: FilterBank's
// constructor sets bank(this), and Controller resets it to null so that
// getBaseFilterbank() terminates (see finding A10).
TEST(ControllerLifecycle, BaseBankHasNoParentBank)
{
    Controller c;

    ASSERT_NE(nullptr, c.baseFilterBank);
    EXPECT_EQ(nullptr, c.baseFilterBank->bank())
        << "if this is non-null, getBaseFilterbank() recurses forever (A10)";
}
