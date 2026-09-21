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

// Unit tests for logging policy -- CLEANUP_PLAN P2-8.
//
// Filter::setLoggingLvl() used to call
//     logging::core::get()->set_filter(severity >= _log_lvl)
// i.e. a per-filter method reconfigured the severity filter of the whole
// process. FilterBank::filter() invoked it three times per child per frame, so
// the effective level was whatever the last filter to run happened to want.

#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/trivial.hpp>
#include <boost/make_shared.hpp>
#include <boost/property_tree/ptree.hpp>

#include <toffy/filter.hpp>
#include <toffy/filterbank.hpp>
#include <toffy/frame.hpp>

#include "dummy_filter.hpp"

namespace logging = boost::log;
namespace sinks = boost::log::sinks;
using toffy::Filter;
using toffy::FilterBank;

namespace {

using Sink = sinks::synchronous_sink<sinks::text_ostream_backend>;

/// Installs a sink that captures log records into a string, and removes it
/// again. The global severity filter is left alone -- tests set it explicitly.
class Capture
{
   public:
    Capture()
    {
        auto backend = boost::make_shared<sinks::text_ostream_backend>();
        // The ostringstream is owned by *this, so the stream must not be freed
        // by the shared_ptr.
        backend->add_stream(
            boost::shared_ptr<std::ostream>(&stream, [](std::ostream*) {}));
        backend->auto_flush(true);
        sink = boost::make_shared<Sink>(backend);
        logging::core::get()->add_sink(sink);
    }

    ~Capture() { logging::core::get()->remove_sink(sink); }

    std::string text() const { return stream.str(); }

   private:
    std::ostringstream stream;
    boost::shared_ptr<Sink> sink;
};

boost::property_tree::ptree levelTree(const std::string& key, int value)
{
    boost::property_tree::ptree pt;
    pt.put(key, value);
    return pt;
}

}  // namespace

// dbg must follow the filter's own level, using the named constant rather than
// the old `_log_lvl <= 1` magic number.
TEST(LoggingPolicy, DbgFollowsTheFilterOwnLevel)
{
    Filter f;

    f.updateConfig(levelTree("loglvl", logging::trivial::debug));
    EXPECT_TRUE(f.dbg) << "debug must enable dbg (boost trivial: trace=0, "
                          "debug=1, info=2)";

    f.updateConfig(levelTree("loglvl", logging::trivial::trace));
    EXPECT_TRUE(f.dbg);

    f.updateConfig(levelTree("loglvl", logging::trivial::info));
    EXPECT_FALSE(f.dbg);

    f.updateConfig(levelTree("loglvl", logging::trivial::error));
    EXPECT_FALSE(f.dbg);
}

// updateConfig() used to set _log_lvl without refreshing dbg, so a filter
// configured with <loglvl>debug</loglvl> kept dbg == false until something
// happened to call setLoggingLvl().
TEST(LoggingPolicy, UpdateConfigRefreshesDerivedState)
{
    Filter f;
    ASSERT_FALSE(f.dbg);

    f.updateConfig(levelTree("loglvl", logging::trivial::debug));

    EXPECT_TRUE(f.dbg)
        << "updateConfig set _log_lvl but left dbg stale; this only worked "
           "before because FilterBank::filter() called setLoggingLvl() per "
           "frame, which P2-8 removed";
}

// <loglvl>99</loglvl> used to be static_cast straight into the severity enum.
TEST(LoggingPolicy, ConfiguredLevelIsClampedToAValidSeverity)
{
    Filter f;

    f.updateConfig(levelTree("loglvl", 99));
    EXPECT_EQ(f.logLvl(), logging::trivial::fatal)
        << "out-of-range config produced a severity above fatal";

    Filter g;
    g.updateConfig(levelTree("loglvl", -5));
    EXPECT_EQ(g.logLvl(), logging::trivial::trace)
        << "out-of-range config produced a severity below trace";
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
    pt.put("loglvl", static_cast<int>(logging::trivial::error));
    pt.put("options.loglvl", static_cast<int>(logging::trivial::debug));

    f.updateConfig(pt);

    EXPECT_EQ(f.logLvl(), logging::trivial::debug)
        << "options.loglvl must win over loglvl, as before the P2-8 rewrite";
}

TEST(LoggingPolicy, SetLogLevelParsesNamesAndDefaultsToInfo)
{
    Filter f;

    f.setLogLevel("debug");
    EXPECT_EQ(f.logLvl(), logging::trivial::debug);
    EXPECT_TRUE(f.dbg);

    f.setLogLevel("warning");
    EXPECT_EQ(f.logLvl(), logging::trivial::warning);

    f.setLogLevel("not-a-level");
    EXPECT_EQ(f.logLvl(), logging::trivial::info)
        << "unparseable level must fall back to info";
}

// The headline regression: running a bank whose child wants a chatty level must
// not relax the process-wide filter. Before P2-8 the child's setLoggingLvl()
// dropped the threshold and the debug record below leaked out.
TEST(LoggingPolicy, RunningABankDoesNotRelaxTheGlobalLevel)
{
    Capture capture;
    Filter::setGlobalLogLevel(logging::trivial::info);

    // A child filter that wants everything.
    DummyFilter* dummy = makeDummyFilter();
    dummy->updateConfig(levelTree("loglvl", logging::trivial::trace));
    ASSERT_TRUE(dummy->dbg);

    FilterBank bank;
    bank.add(dummy);

    toffy::Frame in, out;
    ASSERT_TRUE(bank.filter(in, out));
    ASSERT_TRUE(dummy->wasCalled) << "the bank must actually have run the child";

    BOOST_LOG_TRIVIAL(debug) << "LEAKED_DEBUG_RECORD";
    BOOST_LOG_TRIVIAL(warning) << "VISIBLE_WARNING_RECORD";

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
    Filter::setGlobalLogLevel(logging::trivial::debug);

    BOOST_LOG_TRIVIAL(debug) << "DEBUG_NOW_ALLOWED";

    EXPECT_NE(capture.text().find("DEBUG_NOW_ALLOWED"), std::string::npos);

    Filter::setGlobalLogLevel(logging::trivial::info);
}
