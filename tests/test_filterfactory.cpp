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

// Unit tests for toffy::FilterFactory::createFilter() and its index.
//
// Since P2-5 the factory *creates* filters and *indexes* them, and owns
// neither: createFilter() hands back the only strong reference and the index
// holds a weak one. The ownership consequences for a FilterBank are in
// tests/test_ownership.cpp.

#include <gtest/gtest.h>

#include <toffy/filter.hpp>
#include <toffy/filterfactory.hpp>

#include "dummy_filter.hpp"

using toffy::Filter;
using toffy::FilterFactory;

// An unknown type must fail cleanly. Before the null check on the creator's
// *result* this called a null function pointer and segfaulted, which is how
// findings A18/A20 were first observed in practice rather than on paper.
TEST(FilterFactoryCreate, UnknownTypeReturnsNullAndDoesNotCrash)
{
    // `auto`, not `Filter*`: createFilter() returns a FilterPtr since P2-5, and
    // the test should not have to be rewritten every time the ownership type is
    // spelled differently.
    auto f = FilterFactory::getInstance()->createFilter("no_such_filter_type");
    EXPECT_EQ(nullptr, f);
}

// P0-10: the lookup was `CreateFilterFn fn = creators[type]`. operator[] on a
// map default-constructs a missing key, so a *read* inserted a null entry into
// shared global state on every typo'd type name, and the map grew without
// bound. A failed lookup must leave the registry untouched.
TEST(FilterFactoryCreate, FailedLookupDoesNotRegisterTheType)
{
    ASSERT_FALSE(FilterFactory::hasCreator("no_such_filter_type"));

    EXPECT_EQ(nullptr,
              FilterFactory::getInstance()->createFilter("no_such_filter_type"));

    EXPECT_FALSE(FilterFactory::hasCreator("no_such_filter_type"));
}

// A creator that hands back null must not be dereferenced or registered.
TEST(FilterFactoryCreate, CreatorReturningNullIsRejected)
{
    FilterFactory::registerCreator(
        "nullfactory", []() -> Filter* { return nullptr; });

    EXPECT_EQ(nullptr,
              FilterFactory::getInstance()->createFilter("nullfactory"));
    EXPECT_FALSE(FilterFactory::getInstance()->findFilter("nullfactory"));

    FilterFactory::unregisterCreator("nullfactory");
}

// The happy path still works, and the filter is registered under its id.
TEST(FilterFactoryCreate, RegisteredCreatorProducesRegisteredFilter)
{
    registerDummyFilter();
    auto f = FilterFactory::getInstance()->createFilter("dummy");

    ASSERT_NE(nullptr, f);
    EXPECT_TRUE(FilterFactory::getInstance()->findFilter(f->name()));

    // Nothing has to be handed back any more (P2-5): the caller owns the filter
    // and the index entry is weak, so it expires with it. deleteFilter() is now
    // "forget this name", and it reports that a live filter was registered.
    EXPECT_EQ(1, FilterFactory::getInstance()->deleteFilter(f->name()));
    EXPECT_FALSE(FilterFactory::getInstance()->findFilter(f->name()));

    // Forgetting the name does not destroy the filter: the caller still holds
    // it, and reading through it here is the assertion that it survived.
    EXPECT_FALSE(f->name().empty());
}

// The index entry is weak, so it expires with the filter instead of dangling
// (P2-5). Before this, the registry kept the raw pointer of every filter it had
// ever created and had no way to know the object was gone: findFilter() kept
// answering true and getFilter() handed back freed memory.
TEST(FilterFactoryIndex, EntryExpiresWhenTheOwnerLetsGo)
{
    registerDummyFilter();
    std::string name;
    {
        auto f = FilterFactory::getInstance()->createFilter("dummy");
        ASSERT_NE(nullptr, f);
        name = f->name();
        ASSERT_TRUE(FilterFactory::getInstance()->findFilter(name));
    }

    EXPECT_FALSE(FilterFactory::getInstance()->findFilter(name))
        << "the index still advertises a filter nobody owns";
    EXPECT_EQ(nullptr, FilterFactory::getInstance()->getFilter(name));
}

// deleteFilter() on an entry whose filter is gone reports "not a filter" rather
// than 1, and erases the stale slot on the way out.
TEST(FilterFactoryIndex, ExpiredEntryIsReportedAsMissingAndErased)
{
    registerDummyFilter();
    std::string name;
    {
        auto f = FilterFactory::getInstance()->createFilter("dummy");
        name = f->name();
    }

    EXPECT_EQ(0, FilterFactory::getInstance()->deleteFilter(name));
    EXPECT_EQ(0, FilterFactory::getInstance()->deleteFilter(name));
}
