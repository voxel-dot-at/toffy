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

// Unit tests for toffy::Player.

#include <gtest/gtest.h>

#include <toffy/player.hpp>

using toffy::Player;

// Regression test for CLEANUP_PLAN P0-11 / finding A16.
//
// Player::stop() was declared in the public header but had no definition
// anywhere in the tree, so any caller got an undefined-reference link error.
// Before the fix this test file does not even build -- that is the failure,
// and it is the reason the bug survived: the declared API could never be
// exercised, so nothing ever found out what it did.
TEST(PlayerStop, StopIsDefined)
{
    Player p;

    // stop() is the documented counterpart of run(); it must at least link.
    EXPECT_NO_THROW(p.stop());
}

// stop() before anything was ever run must be a no-op rather than a throw.
// Controller::stop() called _thread.join() unconditionally, and joining a
// non-joinable boost::thread throws thread_resource_error (finding A12).
// Only that one guard is fixed here, because defining Player::stop() without
// it would newly expose the fault on a path no caller could reach before.
TEST(PlayerStop, StopBeforeRunIsSafe)
{
    Player p;

    ASSERT_NO_THROW(p.stop());
    EXPECT_NO_THROW(p.stop()) << "stop() must be idempotent";
}
