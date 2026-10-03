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
#include <vector>

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

// ---------------------------------------------------------------------------
// N5 / P3-6: the file-name patterns are printf format strings, and the CSV
// reads were unchecked.
//
// <options/depthPattern> and <options/amplPattern> come from XML and are passed
// as the *format* argument of snprintf(path, size, pattern, sequence). Anything
// other than one signed-decimal conversion is undefined behaviour, and a short
// or malformed CSV used to write the still-uninitialised loop variable into the
// image while the filter reported success.
//
// The fixtures d_short.csv / a_short.csv hold 2 of the 4 values, d_junk.csv
// starts with a non-number.
// ---------------------------------------------------------------------------

namespace {

/** Point an already-configured source at another pair of files. */
void retarget(toffy::capturers::CSVSource& src, const std::string& stem)
{
    const std::string dir = TOFFY_TEST_CSV_DIR;
    boost::property_tree::ptree pt = config(false);
    pt.put("options.depthPattern", dir + "/" + stem + ".csv");
    pt.put("options.amplPattern", dir + "/" + stem + ".csv");
    src.updateConfig(pt);
}

/** Every depth value in a 2x2 frame. */
std::vector<float> depthValues(toffy::Frame& frame)
{
    const auto d = frame.getMatPtr("depth");
    return {d->at<float>(0, 0), d->at<float>(0, 1), d->at<float>(1, 0),
            d->at<float>(1, 1)};
}

}  // namespace

TEST(CsvSourcePattern, ValidPatternsAreAccepted)
{
    toffy::capturers::CSVSource src;
    boost::property_tree::ptree pt = config(false);
    EXPECT_EQ(1, src.loadConfig(pt));

    pt.put("options.depthPattern", std::string(TOFFY_TEST_CSV_DIR) + "/d_%d.csv");
    pt.put("options.amplPattern", std::string(TOFFY_TEST_CSV_DIR) + "/a_%09i.csv");
    EXPECT_EQ(1, src.loadConfig(pt))
        << "%d and %i with flags/width are exactly what the pattern is for";
}

TEST(CsvSourcePattern, FormatStringThatIsNotAnIntConversionIsRejected)
{
    toffy::capturers::CSVSource src;
    ASSERT_EQ(1, src.loadConfig(config(false)));

    // "%s" would take a const char* from a stack slot that holds an int: a
    // crash or an arbitrary memory read, not a wrong file name.
    boost::property_tree::ptree pt = config(false);
    pt.put("options.depthPattern", std::string("%s%s"));
    EXPECT_EQ(0, src.loadConfig(pt)) << "a two-conversion, wrong-type pattern "
                                        "must be reported at config time";
}

TEST(CsvSourcePattern, RejectedPatternIsNotUsed)
{
    // The rejection has to be observable in the data, not only in the log: the
    // previously configured, valid pattern must still be the one in use.
    toffy::capturers::CSVSource src;
    ASSERT_EQ(1, src.loadConfig(config(false)));

    boost::property_tree::ptree bad = config(false);
    bad.put("options.depthPattern", std::string("%s%s"));
    bad.put("options.amplPattern", std::string("%n"));
    src.loadConfig(bad);

    toffy::Frame frame;
    EXPECT_FLOAT_EQ(0.0f, readDepth(src, frame))
        << "the rejected pattern was used anyway - the frame does not come "
           "from the file that was accepted";
}

TEST(CsvSourceRead, ShortFileStopsInsteadOfWritingUninitialisedValues)
{
    toffy::capturers::CSVSource src;
    ASSERT_EQ(1, src.loadConfig(config(false)));

    toffy::Frame frame;
    ASSERT_TRUE(src.filter(frame, frame));  // complete 2x2 fixture: all 0

    retarget(src, "d_short");  // only 2 of the 4 values exist
    EXPECT_TRUE(src.filter(frame, frame))
        << "a short file is an error to report, not a reason to abort the bank";

    const std::vector<float> d = depthValues(frame);
    EXPECT_FLOAT_EQ(7.0f, d[0]);
    EXPECT_FLOAT_EQ(8.0f, d[1]);
    EXPECT_FLOAT_EQ(0.0f, d[2])
        << "pixel 2 was written from an uninitialised variable: fscanf failed "
           "and its return value was discarded";
    EXPECT_FLOAT_EQ(0.0f, d[3])
        << "pixel 3 was written from an uninitialised variable";
}

TEST(CsvSourceRead, MalformedFirstValueStopsTheRead)
{
    toffy::capturers::CSVSource src;
    ASSERT_EQ(1, src.loadConfig(config(false)));

    toffy::Frame frame;
    ASSERT_TRUE(src.filter(frame, frame));  // all 0

    retarget(src, "d_junk");  // "abc;0;0;0;"
    EXPECT_TRUE(src.filter(frame, frame));

    const std::vector<float> d = depthValues(frame);
    for (size_t i = 0; i < d.size(); i++)
    {
        SCOPED_TRACE("value " + std::to_string(i));
        EXPECT_FLOAT_EQ(0.0f, d[i])
            << "the read stopped at the first value, so nothing may change";
    }
}

TEST(CsvSourceRead, TruncatedPathIsReportedAndTheFrameIsLeftAlone)
{
    toffy::capturers::CSVSource src;
    ASSERT_EQ(1, src.loadConfig(config(false)));

    // Point at frame 1 first, so "unchanged" is a value we can tell apart from
    // "read from the truncated path".
    {
        const std::string dir = TOFFY_TEST_CSV_DIR;
        boost::property_tree::ptree pt = config(false);
        pt.put("options.depthPattern", dir + "/d_00001.csv");
        pt.put("options.amplPattern", dir + "/a_00001.csv");
        src.updateConfig(pt);
    }
    toffy::Frame frame;
    ASSERT_TRUE(src.filter(frame, frame));
    ASSERT_FLOAT_EQ(1.0f, frame.getMatPtr("depth")->at<float>(0, 0));

    // A pattern that is valid but expands past the 1024-byte buffer. Pre-fix
    // the truncated name was opened as if it were the intended file.
    boost::property_tree::ptree pt = config(false);
    pt.put("options.depthPattern",
           std::string(2000, 'x') + "/d_%d.csv");
    src.updateConfig(pt);
    EXPECT_TRUE(src.filter(frame, frame));

    EXPECT_FLOAT_EQ(1.0f, frame.getMatPtr("depth")->at<float>(0, 0))
        << "a truncated path was used as a file name";
}
