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

// Unit tests for toffy::FilterBank.

#include <gtest/gtest.h>

#include <toffy/filterbank.hpp>

#include "dummy_filter.hpp"

using toffy::FilterBank;

TEST(FilterBankSmoke, NewBankIsEmpty)
{
    FilterBank fb;
    EXPECT_EQ(0u, fb.size());
}

// Regression test for CLEANUP_PLAN P0-1 / finding A1:
// FilterBank::stop() called start() on every child, so no filter ever saw a
// stop and every child stayed in filterRunning forever.
TEST(FilterBankStop, StopStopsChildren)
{
    FilterBank fb;
    DummyFilter* dummy = new DummyFilter("dummy");
    fb.add(dummy);

    fb.init();
    fb.start();
    ASSERT_EQ(toffy::filterRunning, dummy->getState());

    fb.stop();

    EXPECT_EQ(toffy::filterIdle, dummy->getState());
    EXPECT_EQ(toffy::filterIdle, fb.getState());
}
