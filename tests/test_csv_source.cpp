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

// Fence for A24: CSVSource conflated its frame counter with its sequence flag.
//
// CSVSource has two members that both answer to the config key
// <options/sequence>: `int sequence`, the index substituted into the file-name
// pattern, and `bool useSequence`, whether that index advances. loadConfig()
// assigned the flag into the counter (`sequence = pt.get<bool>(...)`) and never
// touched useSequence; updateConfig() assigned it into the flag; getConfig()
// wrote the counter out under the flag's key. Neither member had an
// initialiser, so `if (useSequence) sequence++;` in filter() read an
// indeterminate bool.
//
// The observable defect this pins: with <options/sequence> true, the first
// frame must be frame 0 and the second frame 1. Pre-fix the first read is
// frame 1 (the flag landed in the counter) and whether it advances at all
// depends on uninitialised memory.
//
// Fixture: tests/xml/csv/{d,a}_0000N.csv, each 2x2, every value N (ampl N+10),
// so a value in the frame identifies which file was read.

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>

#include <toffy/frame.hpp>
#include <toffy/io/csv_source.hpp>

#ifndef TOFFY_TEST_CSV_DIR
#define TOFFY_TEST_CSV_DIR "."
#endif

namespace {

boost::property_tree::ptree config(bool sequence)
{
    const std::string dir = TOFFY_TEST_CSV_DIR;
    boost::property_tree::ptree pt;
    pt.put("options.width", 2);
    pt.put("options.height", 2);
    pt.put("options.amplPattern", dir + "/a_%05d.csv");
    pt.put("options.depthPattern", dir + "/d_%05d.csv");
    pt.put("options.depth", "depth");
    pt.put("options.ampl", "ampl");
    pt.put("outputs.depth", "depth");
    pt.put("outputs.ampl", "ampl");
    pt.put("options.sequence", sequence);
    return pt;
}

/** Value of the top-left depth pixel after one pass of the filter. */
float readDepth(toffy::capturers::CSVSource& src, toffy::Frame& frame)
{
    EXPECT_TRUE(src.filter(frame, frame));
    return frame.getMatPtr("depth")->at<float>(0, 0);
}

}  // namespace

TEST(CsvSourceSequence, SequencedPlaybackStartsAtFrameZero)
{
    toffy::capturers::CSVSource src;
    src.loadConfig(config(true));

    toffy::Frame frame;
    EXPECT_FLOAT_EQ(0.0f, readDepth(src, frame))
        << "the first frame read was not frame 0 - <options/sequence> is "
           "landing in the frame counter instead of the sequence flag";
}

TEST(CsvSourceSequence, SequencedPlaybackAdvancesTheFrameCounter)
{
    toffy::capturers::CSVSource src;
    src.loadConfig(config(true));

    toffy::Frame frame;
    readDepth(src, frame);
    EXPECT_FLOAT_EQ(1.0f, readDepth(src, frame))
        << "the second pass did not advance to frame 1 - the sequence flag "
           "never reached useSequence, so filter() branched on uninitialised "
           "memory";
}

TEST(CsvSourceSequence, AmplAndDepthAdvanceTogether)
{
    toffy::capturers::CSVSource src;
    src.loadConfig(config(true));

    toffy::Frame frame;
    ASSERT_TRUE(src.filter(frame, frame));
    ASSERT_FLOAT_EQ(10.0f, static_cast<float>(
        frame.getMatPtr("ampl")->at<short>(0, 0)));
    ASSERT_TRUE(src.filter(frame, frame));
    EXPECT_FLOAT_EQ(11.0f, static_cast<float>(
        frame.getMatPtr("ampl")->at<short>(0, 0)));
}

TEST(CsvSourceSequence, WithoutTheFlagTheSameFrameIsReRead)
{
    // The other half of the flag: with sequence off, every pass must re-read
    // the same file. Pre-fix this passes for the wrong reason - useSequence is
    // uninitialised, so "off" is whatever the heap happened to contain.
    toffy::capturers::CSVSource src;
    src.loadConfig(config(false));

    toffy::Frame frame;
    for (int i = 0; i < 3; i++)
    {
        SCOPED_TRACE("pass " + std::to_string(i));
        EXPECT_FLOAT_EQ(0.0f, readDepth(src, frame));
    }
}
