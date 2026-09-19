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

// Unit tests for toffy::Filter construction.

#include <gtest/gtest.h>

#include <boost/log/trivial.hpp>
#include <boost/property_tree/ptree.hpp>

#include <toffy/filter.hpp>

using toffy::Filter;
using toffy::filterLoaded;

// Regression test for CLEANUP_PLAN P0-5 / finding A6.
//
// Filter::Filter() was `Filter::Filter() : _type("...") {}`, which left _bank,
// _log_lvl, dbg and update indeterminate. Both constructors also omitted
// `state` entirely, so getState() before init() returned garbage.
//
// _log_lvl is the observable one: boost::log::trivial::info is 2, so an
// uninitialised member reads as something else and this fails rather than
// passing by luck on zeroed memory. bank() matters most in production:
// Filter::loadGlobals() static_casts it to FilterBank* and calls through it.
TEST(FilterConstruction, DefaultConstructedMembersAreInitialised)
{
    Filter f;

    EXPECT_EQ(nullptr, f.bank())
        << "uninitialised _bank is cast to FilterBank* and dereferenced by "
        << "Filter::loadGlobals()";
    EXPECT_EQ(boost::log::trivial::info, f.logLvl());
    EXPECT_FALSE(f.dbg);
    EXPECT_FALSE(f.update);
    EXPECT_EQ(filterLoaded, f.getState());
}

namespace {
// Filter(std::string, size_t) is protected, so reaching it needs a subclass.
class ProbeFilter : public Filter
{
   public:
    ProbeFilter() : Filter("probefilter") {}
};
}  // namespace

// The typed constructor must give the same guarantees, and must not leave
// `state` unset the way both constructors used to.
TEST(FilterConstruction, TypedConstructorInitialisesMembers)
{
    ProbeFilter f;

    EXPECT_EQ(nullptr, f.bank());
    EXPECT_EQ(boost::log::trivial::info, f.logLvl());
    EXPECT_FALSE(f.dbg);
    EXPECT_FALSE(f.update);
    EXPECT_EQ(filterLoaded, f.getState());
    EXPECT_EQ("probefilter", f.type());
    EXPECT_FALSE(f.id().empty());
    EXPECT_EQ(f.id(), f.name());
}

// Regression tests for CLEANUP_PLAN P0-12 / finding A7.
//
// Filter::loadConfig() looked the type node up with pt.find(), logged a
// detailed diagnostic when it was absent -- and then fell straight through
// into an unguarded pt.get_child(_type), which throws ptree_bad_path. The
// careful error message was therefore always followed by an exception, so
// callers using the documented int return code never saw it.
TEST(FilterLoadConfig, MissingTypeNodeReportsFailureInsteadOfThrowing)
{
    ProbeFilter f;

    boost::property_tree::ptree pt;
    pt.put("some_other_filter.name", "mislabelled");

    int ret = 1;
    ASSERT_NO_THROW(ret = f.loadConfig(pt))
        << "diagnosed the missing node, then threw ptree_bad_path anyway";
    EXPECT_LE(ret, 0)
        << "filter.hpp documents 'positive on success, negative or 0 if failed'";
}

// The guard must not reject the well-formed case: a node named after the
// filter's type still loads and still sets the name from config.
TEST(FilterLoadConfig, MatchingTypeNodeStillSucceeds)
{
    ProbeFilter f;

    boost::property_tree::ptree pt;
    pt.put("probefilter.name", "probe1");

    EXPECT_GT(f.loadConfig(pt), 0);
    EXPECT_EQ("probe1", f.name());
}
