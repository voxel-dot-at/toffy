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

// Characterisation tests for toffy::log -- cleanup/plan/logging.md P2-17.
//
// These exercise the abstraction only: no Boost, no log4cplus. That is the
// point. The same file has to pass unchanged when src/logging.cpp is rewritten
// against log4cplus in step C, so anything here that could only be true of
// Boost.Log is a bug in this test, not a fact about logging.
//
// What is deliberately NOT tested here is what reached a sink. Observing that
// needs a sink, which means naming a backend; test_logging.cpp does it for the
// Filter half and moves at step C.

#include <gtest/gtest.h>

#include <string>

#include <toffy/logging.hpp>

using toffy::log::config;
using toffy::log::level;
using toffy::log::logger;

namespace {

/// Unique per test: channels are process-wide, so two tests sharing a name
/// would see each other's pinned levels.
std::string channelName(const std::string& leaf)
{
    return "toffytest." + leaf;
}

}  // namespace

// The numbers are a wire format: Filter writes logLevel() into pipeline config
// as an integer and reads it back. Renumbering this enum silently re-levels
// every deployed pipeline file, so it is pinned here rather than trusted.
TEST(Level, WireFormatIsFrozen)
{
    EXPECT_EQ(0, static_cast<int>(level::trace));
    EXPECT_EQ(1, static_cast<int>(level::debug));
    EXPECT_EQ(2, static_cast<int>(level::info));
    EXPECT_EQ(3, static_cast<int>(level::warning));
    EXPECT_EQ(4, static_cast<int>(level::error));
    EXPECT_EQ(5, static_cast<int>(level::fatal));
    EXPECT_EQ(6, static_cast<int>(level::off));
}

TEST(Level, NamesRoundTripThroughParse)
{
    const level all[] = {level::trace, level::debug,    level::info,
                         level::warning, level::error,  level::fatal,
                         level::off};
    for (level lvl : all) {
        EXPECT_EQ(lvl, toffy::log::parseLevel(toffy::log::name(lvl)));
    }
}

TEST(Level, ParseIsCaseInsensitive)
{
    EXPECT_EQ(level::debug, toffy::log::parseLevel("DEBUG"));
    EXPECT_EQ(level::debug, toffy::log::parseLevel("Debug"));
    EXPECT_EQ(level::warning, toffy::log::parseLevel("Warning"));
}

// Pipeline files use "warn"; dropping the alias would silently downgrade every
// one of them to the fallback.
TEST(Level, WarnIsAnAliasForWarning)
{
    EXPECT_EQ(level::warning, toffy::log::parseLevel("warn"));
}

TEST(Level, UnparseableFallsBackToTheCallerSChoice)
{
    EXPECT_EQ(level::info, toffy::log::parseLevel("chatty"));
    EXPECT_EQ(level::error, toffy::log::parseLevel("", level::error));
    EXPECT_EQ(level::error, toffy::log::parseLevel("debugg", level::error))
        << "a prefix of a real level must not match it";
}

TEST(Channel, RootIsUnnamedAndStable)
{
    const logger& r = toffy::log::root();
    EXPECT_TRUE(r.name().empty());
    EXPECT_EQ(&r, &toffy::log::root());
    EXPECT_EQ(&toffy::log::root(), &logger::root());
}

TEST(Channel, GetReturnsTheSameObjectForTheSameName)
{
    const std::string name = channelName("stable");
    const logger& a = toffy::log::getLogger(name);
    const logger& b = toffy::log::getLogger(name);
    EXPECT_EQ(&a, &b) << "callers cache the reference; a new object per call "
                         "would make setLevel() on one handle invisible to the "
                         "other";
    EXPECT_EQ(name, a.name());
}

TEST(Channel, EmptyNameIsTheRoot)
{
    EXPECT_EQ(&toffy::log::root(), &toffy::log::getLogger(""));
}

TEST(Channel, AChannelInheritsTheRootUntilItIsPinned)
{
    const std::string name = channelName("inherit");
    const logger& c = toffy::log::getLogger(name);
    EXPECT_EQ(toffy::log::root().effectiveLevel(), c.effectiveLevel());
    EXPECT_EQ(toffy::log::root().enabled(level::debug),
              c.enabled(level::debug));
}

TEST(Channel, PinningAChannelDoesNotTouchItsParentOrTheRoot)
{
    const std::string leaf = channelName("pin");
    const std::string childName = leaf + ".child";
    const logger& parent = toffy::log::getLogger(leaf);
    const logger& child = toffy::log::getLogger(childName);

    const level before = toffy::log::root().effectiveLevel();
    parent.setLevel(level::error);

    EXPECT_EQ(level::error, parent.effectiveLevel());
    EXPECT_EQ(level::error, child.effectiveLevel())
        << "a child must follow its parent";
    EXPECT_EQ(before, toffy::log::root().effectiveLevel())
        << "pinning one channel silenced the process";

    parent.inheritLevel();
    EXPECT_EQ(before, parent.effectiveLevel());
}

// log4cplus creates the intermediate loggers of a dotted name implicitly, so
// pinning "toffytest" applies to "toffytest.mid.leaf" even though nothing ever
// asked for "toffytest.mid". The interim backend has to agree or the swap
// changes which records appear.
TEST(Channel, InheritanceSkipsOverAncestorsThatWereNeverRequested)
{
    const std::string top = channelName("gap");
    const std::string leafName = top + ".mid.leaf";
    const logger& grandparent = toffy::log::getLogger(top);
    const logger& leaf = toffy::log::getLogger(leafName);

    grandparent.setLevel(level::error);
    EXPECT_EQ(level::error, leaf.effectiveLevel())
        << "the walk stopped at the missing \"mid\" instead of continuing up";

    grandparent.inheritLevel();
    EXPECT_EQ(toffy::log::root().effectiveLevel(), leaf.effectiveLevel());
}

TEST(Channel, AChannelCanSilenceItselfAgainstTheRoot)
{
    const std::string name = channelName("quiet");
    const logger& c = toffy::log::getLogger(name);
    c.setLevel(level::off);
    EXPECT_FALSE(c.enabled(level::fatal));
    EXPECT_TRUE(toffy::log::root().enabled(level::fatal))
        << "\"this filter is quiet\" must not mean \"logging is off\"";
    c.inheritLevel();
}

// The rule the interim backend implements on purpose: toffy's layer decides,
// and Boost's own severity filter is left wide open. log4cplus has per-logger
// levels natively and will keep this behaviour, so it is a contract, not a
// workaround -- which is why it is asserted rather than noted.
TEST(Channel, AChannelCanAskForMoreThanTheRootAllows)
{
    const level rootBefore = toffy::log::root().effectiveLevel();
    toffy::log::root().setLevel(level::error);

    const std::string name = channelName("verbose");
    const logger& c = toffy::log::getLogger(name);
    c.setLevel(level::debug);

    EXPECT_FALSE(toffy::log::root().enabled(level::debug));
    EXPECT_TRUE(c.enabled(level::debug));

    c.inheritLevel();
    toffy::log::root().setLevel(rootBefore);
}

TEST(Init, InitOnceLeavesAnExistingConfigurationAlone)
{
    // Player's ctor calls initOnce(), so an application that configured logging
    // in main() first must not have its choice replaced by the Player default.
    config first;
    first.minLevel = level::error;
    toffy::log::init(first);
    EXPECT_TRUE(toffy::log::isConfigured());
    EXPECT_EQ(level::error, toffy::log::root().effectiveLevel());

    config second;
    second.minLevel = level::trace;
    toffy::log::initOnce(second);
    EXPECT_EQ(level::error, toffy::log::root().effectiveLevel())
        << "initOnce() overwrote a configuration that was already applied";

    // init(), by contrast, always applies.
    toffy::log::init(second);
    EXPECT_EQ(level::trace, toffy::log::root().effectiveLevel());

    toffy::log::init(first);
}

TEST(Init, EnabledFollowsTheConfiguredLevel)
{
    config cfg;
    cfg.minLevel = level::warning;
    toffy::log::init(cfg);

    EXPECT_FALSE(toffy::log::enabled(level::debug));
    EXPECT_FALSE(toffy::log::enabled(level::info));
    EXPECT_TRUE(toffy::log::enabled(level::warning));
    EXPECT_TRUE(toffy::log::enabled(level::fatal));

    cfg.minLevel = level::debug;
    toffy::log::init(cfg);
    EXPECT_TRUE(toffy::log::enabled(level::debug));
}
