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

#include <toffy/controller.hpp>

using toffy::Controller;

// Regression tests for CLEANUP_PLAN P0-9 / finding C3.
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
// the guards do not fire spuriously and that the lifecycle -- including the
// process-global side ~Controller has on the creator registry -- still works.
TEST(ControllerLifecycle, ConstructAndDestroy)
{
    ASSERT_NO_THROW({
        Controller c;
        EXPECT_NE(nullptr, c.baseFilterBank);
    });
}

// ~Controller calls FilterFactory::clearCreators(), which wipes the *process
// wide* creator registry. A second Controller in the same process must still be
// able to obtain its base bank. This passes because the built-in types resolve
// through createFilter()'s hard-coded chain rather than the creator map -- the
// coupling is real but currently benign, and this test says so in code.
TEST(ControllerLifecycle, SequentialControllersStillWork)
{
    ASSERT_NO_THROW({ Controller first; });
    ASSERT_NO_THROW({
        Controller second;
        EXPECT_NE(nullptr, second.baseFilterBank);
    });
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
