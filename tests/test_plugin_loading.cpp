/*
 * toffy cleanup -- P2-10, the three plugin loaders.
 *
 * The same feature was implemented three times, with three different failure
 * modes:
 *
 *   Player::loadPlugins      looks up "toffy.plugins", catches, logs a warning
 *   Controller::loadPlugins  looks up "toffy.plugins", THROWS ptree_bad_path
 *   FilterBank::loadPlugins  looks up "plugins"           THROWS, and discards
 *                                                          the dlopen handle
 *
 * The throw matters because a config with no <plugins> node is normal:
 * becom-knapp-wurthbox's own wbox.xml has none at all (testbox.xml has an empty
 * one). Only Player::loadConfig() is on the live path, and it is the one that
 * catches -- which is why production works and why the other two have never
 * shown up as a failure. They are still public API, and still wrong.
 *
 * Copyright 2026 Simon Vogl <svogl@voxel.at>
 * Licensed under the Apache License, Version 2.0.
 */

#include <string>

#include <gtest/gtest.h>

#include <boost/property_tree/ptree.hpp>
#include <log4cplus/callbackappender.h>
#include <log4cplus/logger.h>

#include <toffy/controller.hpp>
#include <toffy/filterbank.hpp>

using boost::property_tree::ptree;

namespace {

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

/// Captures log records into a string and removes only its own appender again.
/// Same shape as tests/test_logging.cpp; duplicated rather than shared because
/// test targets are deliberately independent.
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

/// A config with a <plugins> node naming a library that does not exist. The
/// point is not to load anything: it is to have something in the node that the
/// loader will visibly try, so "did it even find the node" becomes observable.
ptree configWithOnePlugin(const std::string& lib)
{
    ptree pt;
    ptree plugins;
    plugins.add("plugin", lib);
    pt.add_child("toffy.plugins", plugins);
    return pt;
}

/// A config with no <plugins> node at all -- the shape of wbox.xml.
ptree configWithoutPlugins()
{
    ptree pt;
    pt.put("toffy.globals.options.something", "1");
    return pt;
}

const std::string kBogus = "toffy-p210-definitely-not-a-real-plugin.so";

/// FilterBank::loadPlugins is protected, so a test needs a subclass to reach it.
/// Worth noting: protected-and-never-called is a different thing from public API.
/// Nothing in toffy or either consumer subclasses FilterBank except Cond and
/// ParallelFilter, and neither calls this, so consolidating it is not the API
/// break the plan assumed it would have to be.
class ExposedFilterBank : public toffy::FilterBank
{
   public:
    using toffy::FilterBank::loadPlugins;
};

}  // namespace

// ---------------------------------------------------------------------------

TEST(PluginLoadingP210, MissingPluginsNodeDoesNotThrowViaController)
{
    toffy::Controller c;
    // ptree::get_child() throws ptree_bad_path on a missing path. A config that
    // simply has no plugins is not an error, and boxes' wbox.xml is one.
    EXPECT_NO_THROW(c.loadPlugins(configWithoutPlugins()));
}

TEST(PluginLoadingP210, MissingPluginsNodeDoesNotThrowViaFilterBank)
{
    ExposedFilterBank fb;
    EXPECT_NO_THROW(fb.loadPlugins(configWithoutPlugins()));
}

TEST(PluginLoadingP210, FilterBankFindsTheSameNodeTheOthersDo)
{
    // The node-name split was the silent half of the bug: FilterBank looked up
    // "plugins" while every config in the tree nests it as /toffy/plugins, so it
    // could never find what the other two found. This asserts the loop actually
    // ran over the entry, by observing that it tried to dlopen it.
    ExposedFilterBank fb;
    Capture cap;
    fb.loadPlugins(configWithOnePlugin(kBogus));

    EXPECT_NE(std::string::npos, cap.text().find(kBogus))
        << "FilterBank::loadPlugins did not visit the <plugins> node that "
           "Player and Controller find; it is still looking for a differently "
           "named node";
}

TEST(PluginLoadingP210, UnloadableLibraryIsAWarningAndNotACrash)
{
    toffy::Controller c;
    Capture cap;
    EXPECT_NO_THROW(c.loadPlugins(configWithOnePlugin(kBogus)));
    EXPECT_NE(std::string::npos, cap.text().find(kBogus))
        << "a plugin that cannot be loaded should at least be named in the log";
}

TEST(PluginLoadingP210, EmptyPluginsNodeIsFineEverywhere)
{
    // testbox.xml's shape: <plugins></plugins>. Present but empty.
    ptree pt;
    pt.add_child("toffy.plugins", ptree());

    toffy::Controller c;
    ExposedFilterBank fb;
    EXPECT_NO_THROW(c.loadPlugins(pt));
    EXPECT_NO_THROW(fb.loadPlugins(pt));
}

TEST(PluginLoadingP210, ARealPluginLoadsAndWhatItRegistersIsUsable)
{
    // The end-to-end case. Everything above this exercises the failure branch;
    // this is the only test in the tree that dlopens a plug-in that exists and
    // checks that init() actually ran and that what it registered is reachable.
    toffy::FilterFactory::getInstance();  // warm, so hasCreator() means what it says
    ASSERT_FALSE(toffy::FilterFactory::hasCreator("test.p210plugin"))
        << "the fixture registered itself without being loaded";

    const size_t before = toffy::commons::plugins::loadedHandles().size();

    toffy::Controller c;
    c.loadPlugins(configWithOnePlugin(TOFFY_TEST_PLUGIN));

    EXPECT_TRUE(toffy::FilterFactory::hasCreator("test.p210plugin"))
        << "the plug-in was not loaded, or init() was not called";

    // Single owner: the handle is recorded, not discarded. This is the specific
    // thing FilterBank::loadPlugins used to get wrong.
    EXPECT_EQ(before + 1, toffy::commons::plugins::loadedHandles().size())
        << "a successfully loaded plug-in handle was not recorded by the loader";
}

TEST(PluginLoadingP210, FilterBankPathRecordsTheHandleToo)
{
    // The same fixture through the FilterBank entry point, which is the one that
    // used to throw its handles away. If this regresses, the leak comes back for
    // that caller only -- which is exactly the kind of divergence three copies
    // produced.
    toffy::FilterFactory::getInstance();
    const size_t before = toffy::commons::plugins::loadedHandles().size();

    ExposedFilterBank fb;
    fb.loadPlugins(configWithOnePlugin(TOFFY_TEST_PLUGIN));

    EXPECT_EQ(before + 1, toffy::commons::plugins::loadedHandles().size());
}
