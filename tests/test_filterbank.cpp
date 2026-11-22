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
#include <memory>

#include <gtest/gtest.h>

#include <toffy/filterbank.hpp>

#include "dummy_filter.hpp"

using toffy::FilterBank;

TEST(FilterBankSmoke, NewBankIsEmpty)
{
    FilterBank fb;
    EXPECT_EQ(0u, fb.size());
}

// Regression test for cleanup/plan/p0-correctness.md P0-1 / finding A1:
// FilterBank::stop() called start() on every child, so no filter ever saw a
// stop and every child stayed in filterRunning forever.
TEST(FilterBankStop, StopStopsChildren)
{
    FilterBank fb;
    auto dummy = makeDummyFilter();
    fb.add(dummy);

    fb.init();
    fb.start();
    ASSERT_EQ(toffy::filterRunning, dummy->getState());

    fb.stop();

    EXPECT_EQ(toffy::filterIdle, dummy->getState());
    EXPECT_EQ(toffy::filterIdle, fb.getState());
}

// Regression tests for cleanup/plan/p0-correctness.md P0-2/P0-3/P0-4 (findings A2, A3, A4).
//
// findPos() returned size_t and "-1" on failure, i.e. SIZE_MAX. remove(name)
// narrowed that into an int and erased at begin() - 1 with no bounds check;
// remove(i) erased at _pipe.end() + i, which is out of range for every i.

TEST(FilterBankFindPos, MissingNameYieldsNoValue)
{
    FilterBank fb;
    EXPECT_FALSE(fb.findPos("missing").has_value());

    auto d = makeDummyFilter();
    fb.add(d);
    ASSERT_TRUE(fb.findPos(d->name()).has_value());
    EXPECT_EQ(0, *fb.findPos(d->name()));
}

TEST(FilterBankRemove, ByIndexRemovesTheElementAtThatIndex)
{
    FilterBank fb;
    auto a = makeDummyFilter();
    auto b = makeDummyFilter();
    fb.add(a);
    fb.add(b);
    ASSERT_EQ(2u, fb.size());

    EXPECT_EQ(1, fb.remove(std::size_t(0)));

    ASSERT_EQ(1u, fb.size());
    EXPECT_EQ(b.get(), fb.getFilter(0));
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
    auto a = makeDummyFilter();
    fb.add(a);

    EXPECT_EQ(0, fb.remove("no-such-filter"));

    ASSERT_EQ(1u, fb.size());
    EXPECT_EQ(a.get(), fb.getFilter(0));
}

// ---------------------------------------------------------------------------
// P2-9 -- FilterBank::getFilter(const std::string&)
//
// Both the const and the non-const overload did `return ff->getFilter(name);`,
// i.e. they asked the process-wide FilterFactory registry. So the answer had
// nothing to do with what this bank contains: a filter created anywhere else in
// the process came back, and a filter this bank holds but that did not arrive
// through FilterFactory::createFilter() was reported missing. The method's own
// @deprecated note asks for exactly this to be reimplemented against _pipe.
// ---------------------------------------------------------------------------

TEST(FilterBankGetFilterByName, ReturnsAFilterTheBankActuallyContains)
{
    FilterBank fb;
    // Deliberately NOT created through the factory, so it is absent from the
    // global registry. The bank holds it, so the bank must be able to name it.
    auto d = std::make_shared<DummyFilter>();
    fb.add(d);

    ASSERT_EQ(1u, fb.size());
    EXPECT_EQ(d.get(), fb.getFilter(d->id()))
        << "the bank holds this filter but getFilter() could not find it: it is "
           "asking the global factory instead of its own pipe";
}

TEST(FilterBankGetFilterByName, DoesNotReturnAFilterTheBankDoesNotContain)
{
    auto elsewhere = makeDummyFilter();  // through the factory: IS in the registry

    FilterBank fb;  // empty
    ASSERT_EQ(0u, fb.size());

    EXPECT_EQ(nullptr, fb.getFilter(elsewhere->id()))
        << "an empty bank returned a filter it does not contain, because the "
           "global registry knows it";
}

TEST(FilterBankGetFilterByName, ConstOverloadAgreesWithTheNonConstOne)
{
    FilterBank fb;
    auto d = std::make_shared<DummyFilter>();
    fb.add(d);

    const FilterBank &cfb = fb;
    EXPECT_EQ(fb.getFilter(d->id()), cfb.getFilter(d->id()))
        << "the const and non-const overloads must not disagree";
}

TEST(FilterBankGetFilterByName, FindsAFilterNestedInASubBank)
{
    // countFiltersByType() and getFiltersByType() both recurse into embedded
    // banks. The old implementation "found" nested filters only by accident --
    // the global registry does not care about bank structure -- so recursing is
    // what preserves the useful half of the old behaviour rather than narrowing it.
    FilterBank outer;
    auto inner = std::make_shared<FilterBank>();
    auto d = std::make_shared<DummyFilter>();
    inner->add(d);
    outer.add(inner);

    EXPECT_EQ(d.get(), outer.getFilter(d->id()))
        << "a filter inside an embedded bank is still reachable from the outer one";
}
