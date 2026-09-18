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

// Unit tests for toffy::FilterFactory::createFilter().

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
    Filter* f =
        FilterFactory::getInstance()->createFilter("no_such_filter_type");
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
    Filter* f = FilterFactory::getInstance()->createFilter("dummy");

    ASSERT_NE(nullptr, f);
    EXPECT_TRUE(FilterFactory::getInstance()->findFilter(f->name()));

    // The factory owns what it created; hand it back so the suite stays
    // leak-free under ASan.
    EXPECT_EQ(1, FilterFactory::getInstance()->deleteFilter(f->name()));
}
