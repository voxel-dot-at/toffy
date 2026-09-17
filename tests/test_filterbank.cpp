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

#include <cstddef>

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
    DummyFilter* dummy = makeDummyFilter();
    fb.add(dummy);

    fb.init();
    fb.start();
    ASSERT_EQ(toffy::filterRunning, dummy->getState());

    fb.stop();

    EXPECT_EQ(toffy::filterIdle, dummy->getState());
    EXPECT_EQ(toffy::filterIdle, fb.getState());
}

// Regression tests for CLEANUP_PLAN P0-2/P0-3/P0-4 (findings A2, A3, A4).
//
// findPos() returned size_t and "-1" on failure, i.e. SIZE_MAX. remove(name)
// narrowed that into an int and erased at begin() - 1 with no bounds check;
// remove(i) erased at _pipe.end() + i, which is out of range for every i.

TEST(FilterBankFindPos, MissingNameYieldsNoValue)
{
    FilterBank fb;
    EXPECT_FALSE(fb.findPos("missing").has_value());

    DummyFilter* d = makeDummyFilter();
    fb.add(d);
    ASSERT_TRUE(fb.findPos(d->name()).has_value());
    EXPECT_EQ(0, *fb.findPos(d->name()));
}

TEST(FilterBankRemove, ByIndexRemovesTheElementAtThatIndex)
{
    FilterBank fb;
    DummyFilter* a = makeDummyFilter();
    DummyFilter* b = makeDummyFilter();
    fb.add(a);
    fb.add(b);
    ASSERT_EQ(2u, fb.size());

    EXPECT_EQ(1, fb.remove(std::size_t(0)));

    ASSERT_EQ(1u, fb.size());
    EXPECT_EQ(static_cast<toffy::Filter*>(b), fb.getFilter(0));
}

TEST(FilterBankRemove, ByIndexOutOfRangeIsRejected)
{
    FilterBank fb;
    EXPECT_EQ(0, fb.remove(std::size_t(0)));
    EXPECT_EQ(0u, fb.size());
}

TEST(FilterBankRemove, MissingNameLeavesBankIntact)
{
    FilterBank fb;
    DummyFilter* a = makeDummyFilter();
    fb.add(a);

    EXPECT_EQ(0, fb.remove("no-such-filter"));

    ASSERT_EQ(1u, fb.size());
    EXPECT_EQ(static_cast<toffy::Filter*>(a), fb.getFilter(0));
}
