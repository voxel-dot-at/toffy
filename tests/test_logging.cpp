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

// Unit tests for logging policy -- cleanup/plan/structure.md P2-8, and the
// parts of cleanup/plan/logging.md P2-17 that Filter owns.
//
// Filter::setLoggingLvl() used to call
//     logging::core::get()->set_filter(severity >= _log_lvl)
// i.e. a per-filter method reconfigured the severity filter of the whole
// process. FilterBank::filter() invoked it three times per child per frame, so
// the effective level was whatever the last filter to run happened to want.
//
// The `Capture` helper below reaches into the logging backend directly. That is
// a deliberate exception: this file tests that *Filter* does not touch the
// process-wide configuration, and the only way to see what reached the sinks is
// to install one. It is the one place in the tree outside src/logging.cpp
// allowed to name the backend. The backend-independent half of the API is tested
// in test_logging_abstraction.cpp instead, and that file has to pass against any
// backend — which is the property that makes this one's coupling acceptable.

#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include <boost/property_tree/ptree.hpp>

#include <log4cplus/callbackappender.h>
#include <log4cplus/clogger.h>
#include <log4cplus/logger.h>

#include <toffy/filter.hpp>
#include <toffy/filterbank.hpp>
#include <toffy/frame.hpp>
#include <toffy/logging.hpp>

#include "dummy_filter.hpp"

using toffy::Filter;
using toffy::FilterBank;
using toffy::log::level;

namespace {

/// Appends each record's message to a string. The cookie is the Capture's own
/// buffer, so nothing is shared between instances and the callback needs no
/// lock: a test thread is the only writer.
void capture_to_string(void* cookie, log4cplus_char_t const* message,
                       log4cplus_char_t const* /*loggerName*/,
                       log4cplus_loglevel_t /*ll*/, log4cplus_char_t const* /*thread*/,
                       log4cplus_char_t const* /*thread2*/,
                       unsigned long long /*timestamp_secs*/,
                       unsigned long /*timestamp_usecs*/,
                       log4cplus_char_t const* /*file*/,
                       log4cplus_char_t const* /*function*/, int /*line*/)
{
    static_cast<std::string*>(cookie)->append(message).push_back('\n');
}

/// Installs an appender that captures log records into a string, and removes
/// that one appender again -- not every appender, which would silence the
/// console for the tests that run after this one.
class Capture
{
   public:
    Capture()
    {
        appender = log4cplus::SharedAppenderPtr(
            new log4cplus::CallbackAppender(&capture_to_string, &text_));
        log4cplus::Logger::getRoot().addAppender(appender);
    }

    ~Capture() { log4cplus::Logger::getRoot().removeAppender(appender); }

    std::string text() const { return text_; }

   private:
    std::string text_;
    log4cplus::SharedAppenderPtr appender;
};

boost::property_tree::ptree levelTree(const std::string& key, int value)
{
    boost::property_tree::ptree pt;
    pt.put(key, value);
    return pt;
}

boost::property_tree::ptree levelTree(const std::string& key, level lvl)
{
    return levelTree(key, static_cast<int>(lvl));
}

}  // namespace

// debugEnabled() must follow the filter's own level, with no cached flag in
// between that a caller has to remember to refresh.
TEST(LoggingPolicy, DebugFlagFollowsTheFilterOwnLevel)
{
    Filter f;

    f.updateConfig(levelTree("loglvl", level::debug));
    EXPECT_TRUE(f.debugEnabled()) << "debug must enable debug output (trace=0, "
                                     "debug=1, info=2)";

    f.updateConfig(levelTree("loglvl", level::trace));
    EXPECT_TRUE(f.debugEnabled());

    f.updateConfig(levelTree("loglvl", level::info));
    EXPECT_FALSE(f.debugEnabled());

    f.updateConfig(levelTree("loglvl", level::error));
    EXPECT_FALSE(f.debugEnabled());
}

// updateConfig() used to set the level without refreshing the cached `dbg`, so
// a filter configured with <loglvl>debug</loglvl> kept dbg == false until
// something happened to call setLoggingLvl(). Deriving the flag on read makes
// the stale state unrepresentable, which is why this test is now about the
// accessor rather than about the refresh.
TEST(LoggingPolicy, UpdateConfigTakesEffectWithoutAnyRefresh)
{
    Filter f;
    ASSERT_FALSE(f.debugEnabled());

    f.updateConfig(levelTree("loglvl", level::debug));

    EXPECT_TRUE(f.debugEnabled())
        << "there is no derived state left to go stale; if this fails, "
           "debugEnabled() stopped reading logLevel()";
}

// <loglvl>99</loglvl> used to be static_cast straight into the severity enum.
TEST(LoggingPolicy, ConfiguredLevelIsClampedToAValidSeverity)
{
    Filter f;

    f.updateConfig(levelTree("loglvl", 99));
    EXPECT_EQ(f.logLevel(), level::fatal)
        << "out-of-range config produced a severity above fatal";

    Filter g;
    g.updateConfig(levelTree("loglvl", -5));
    EXPECT_EQ(g.logLevel(), level::trace)
        << "out-of-range config produced a severity below trace";
}

// `off` is deliberately not reachable from config: silencing a process is an
// application decision, not a pipeline one.
TEST(LoggingPolicy, ConfigCannotReachOff)
{
    Filter f;
    f.updateConfig(levelTree("loglvl", static_cast<int>(level::off)));
    EXPECT_EQ(f.logLevel(), level::fatal)
        << "a pipeline file must not be able to switch logging off";
}

// The two spellings of the setting must keep their original precedence: the
// code applied `loglvl` and then `options.loglvl` with the previous result as
// its default, so options.loglvl wins when both are present.
TEST(LoggingPolicy, OptionsLogLevelStillOverridesLogLevel)
{
    // Stored as ints, which is what an XML <loglvl>4</loglvl> becomes once
    // property_tree parses it. (Handing put() the raw enum does not round-trip
    // through get<int>, which silently returns the default -- that is a ptree
    // quirk, not the behaviour under test.)
    Filter f;
    boost::property_tree::ptree pt;
    pt.put("loglvl", static_cast<int>(level::error));
    pt.put("options.loglvl", static_cast<int>(level::debug));

    f.updateConfig(pt);

    EXPECT_EQ(f.logLevel(), level::debug)
        << "options.loglvl must win over loglvl, as before the P2-8 rewrite";
}

// The integer in the file is the wire format: getConfig() must write the level
// back as the same number updateConfig() read, or a saved pipeline re-levels
// itself on the next load.
TEST(LoggingPolicy, LevelRoundTripsThroughConfigAsAnInteger)
{
    Filter f;
    f.updateConfig(levelTree("loglvl", level::warning));

    const int written = f.getConfig().get<int>("options.loglvl", -1);
    EXPECT_EQ(written, static_cast<int>(level::warning));

    Filter g;
    g.updateConfig(f.getConfig());
    EXPECT_EQ(g.logLevel(), level::warning);
}

TEST(LoggingPolicy, SetLogLevelParsesNamesAndDefaultsToInfo)
{
    Filter f;

    f.setLogLevel("debug");
    EXPECT_EQ(f.logLevel(), level::debug);
    EXPECT_TRUE(f.debugEnabled());

    f.setLogLevel("warning");
    EXPECT_EQ(f.logLevel(), level::warning);

    // "warn" is an alias pipeline files use; parseLevel() has to keep it.
    f.setLogLevel("warn");
    EXPECT_EQ(f.logLevel(), level::warning);

    f.setLogLevel("not-a-level");
    EXPECT_EQ(f.logLevel(), level::info)
        << "unparseable level must fall back to info";
}

// The headline regression: running a bank whose child wants a chatty level must
// not relax the process-wide filter. Before P2-8 the child's setLoggingLvl()
// dropped the threshold and the debug record below leaked out.
TEST(LoggingPolicy, RunningABankDoesNotRelaxTheGlobalLevel)
{
    Capture capture;
    Filter::setGlobalLogLevel(level::info);

    // A child filter that wants everything.
    auto dummy = makeDummyFilter();
    dummy->updateConfig(levelTree("loglvl", level::trace));
    ASSERT_TRUE(dummy->debugEnabled());

    FilterBank bank;
    bank.add(dummy);

    toffy::Frame in, out;
    ASSERT_TRUE(bank.filter(in, out));
    ASSERT_TRUE(dummy->wasCalled) << "the bank must actually have run the child";

    toffy::log::debug() << "LEAKED_DEBUG_RECORD";
    toffy::log::warning() << "VISIBLE_WARNING_RECORD";

    const std::string logged = capture.text();

    EXPECT_EQ(logged.find("LEAKED_DEBUG_RECORD"), std::string::npos)
        << "a filter's own log level changed the severity filter for the whole "
           "process";
    EXPECT_NE(logged.find("VISIBLE_WARNING_RECORD"), std::string::npos)
        << "the capture sink is not working, which makes the assertion above "
           "vacuous";
}

// ...and the converse: setGlobalLogLevel() is the sanctioned way to move it.
TEST(LoggingPolicy, SetGlobalLogLevelDoesAffectOutput)
{
    Capture capture;
    Filter::setGlobalLogLevel(level::debug);

    toffy::log::debug() << "DEBUG_NOW_ALLOWED";

    EXPECT_NE(capture.text().find("DEBUG_NOW_ALLOWED"), std::string::npos);

    Filter::setGlobalLogLevel(level::info);
}
