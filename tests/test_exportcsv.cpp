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

// Fence for N5 / P3-6 in ExportCSV.
//
// ExportCSV used options/pattern as the *format* argument of snprintf:
//
//   char path[_filePattern.length() + 64];
//   snprintf(path, _filePattern.length() + 64, _filePattern.c_str(), _cnt);
//
// The pattern comes straight out of XML. -Wformat cannot help, because the
// format is not a literal, and a pattern whose conversions do not match the one
// int argument is undefined behaviour: "%s%s" reads two pointers out of the
// stack slot that holds the frame counter. The expansion was also unchecked, so
// a result that did not fit the buffer was opened as if it were the intended
// file.
//
// The fix validates at config time (keeping the pattern that already worked)
// and routes every expansion through toffy::formatPath, which reports
// truncation. These tests pin both halves; the second one is the fence and was
// observed crashing on the parent commit.
//
// The same finding's other half, CSVSource, is covered in test_csv_source.cpp.

#include <filesystem>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <toffy/frame.hpp>
#include <toffy/viewers/exportcsv.hpp>

namespace {

/** A <exportcsv> node of the shape FilterBank would hand to loadConfig(). */
boost::property_tree::ptree config(const std::string& pattern, bool sequence,
                                   const std::string& fc = "") {
    boost::property_tree::ptree node, root;
    node.put("options.pattern", pattern);
    node.put("options.sequence", sequence);
    node.put("options.fc", fc);
    node.put("inputs.img", "depth");
    root.add_child("exportcsv", node);
    return root;
}

/** A fresh directory per test, so file existence is unambiguous. */
std::string scratchDir(const std::string& name) {
    const std::string dir = ::testing::TempDir() + "/" + name;
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir);
    return dir;
}

/** Regular files directly inside `dir`. */
std::size_t filesIn(const std::string& dir) {
    std::size_t n = 0;
    for (auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.is_regular_file()) ++n;
    }
    return n;
}

void addDepth(toffy::Frame& frame) {
    frame.addData("depth",
                  toffy::matPtr(new cv::Mat(2, 2, CV_32FC1, cv::Scalar(1))));
}

}  // namespace

TEST(ExportCsvPattern, ValidPatternExpandsTheSequenceCounter)
{
    const std::string dir = scratchDir("sequence");
    toffy::ExportCSV exporter;

    ASSERT_GT(exporter.loadConfig(config(dir + "/out_%d.csv", true)), 0);

    toffy::Frame frame;
    addDepth(frame);
    EXPECT_TRUE(exporter.filter(frame, frame));
    EXPECT_TRUE(exporter.filter(frame, frame));

    EXPECT_TRUE(std::filesystem::exists(dir + "/out_0.csv"));
    EXPECT_TRUE(std::filesystem::exists(dir + "/out_1.csv"));
    EXPECT_EQ(2u, filesIn(dir));
}

TEST(ExportCsvPattern, FormatStringThatIsNotAnIntConversionIsRejected)
{
    const std::string dir = scratchDir("rejected");
    toffy::ExportCSV exporter;
    ASSERT_GT(exporter.loadConfig(config(dir + "/keep_%d.csv", true)), 0);

    // A reconfigure must not be able to install a format that snprintf would
    // misinterpret. Pre-fix this replaced the working pattern, and the next
    // filter() call read pointers off the stack: observed as a segfault on the
    // parent commit, not merely a wrong file name.
    exporter.updateConfig(config("%s%s", true).get_child("exportcsv"));

    toffy::Frame frame;
    addDepth(frame);
    EXPECT_TRUE(exporter.filter(frame, frame));

    EXPECT_TRUE(std::filesystem::exists(dir + "/keep_0.csv"));
    EXPECT_EQ(1u, filesIn(dir));
}

TEST(ExportCsvPattern, ARejectedPatternIsAlsoRejectedAtLoadTime)
{
    const std::string dir = scratchDir("rejected_at_load");
    toffy::ExportCSV exporter;

    // The default pattern ("depth_%d.csv") is valid, so rejecting the supplied
    // one leaves a working pattern in place rather than a format the filter
    // would misread on the first frame.
    exporter.loadConfig(config(dir + "/%f.csv", true));

    EXPECT_EQ("depth_%d.csv",
              exporter.getConfig().get<std::string>("options.pattern"));
    EXPECT_EQ(0u, filesIn(dir));
}

TEST(ExportCsvFrameCounter, PatternUsesTheFrameSlotRatherThanTheInternalCounter)
{
    const std::string dir = scratchDir("framecounter");
    toffy::ExportCSV exporter;

    // sequence off, fc on: the number in the name comes from the frame.
    ASSERT_GT(exporter.loadConfig(config(dir + "/fc_%d.csv", false, "fc")), 0);

    toffy::Frame frame;
    addDepth(frame);
    frame.addData("fc", 42u);
    EXPECT_TRUE(exporter.filter(frame, frame));

    EXPECT_TRUE(std::filesystem::exists(dir + "/fc_42.csv"));
    EXPECT_EQ(1u, filesIn(dir));
}

TEST(ExportCsvPattern, OverlongExpansionWritesNothing)
{
    const std::string dir = scratchDir("overlong");
    toffy::ExportCSV exporter;

    // A pattern that expands past the buffer. This is a pin rather than a
    // fence: the buffer it replaces was a VLA sized from the pattern, so the
    // pre-fix code also wrote no file - it just failed to open a 4 200
    // character name and said nothing about why. What is pinned is that the
    // expansion is bounded and reported, and that the frame is not half
    // processed.
    const std::string pattern = dir + "/" + std::string(4200, 'x') + "_%d.csv";
    exporter.loadConfig(config(pattern, true));

    toffy::Frame frame;
    addDepth(frame);
    EXPECT_TRUE(exporter.filter(frame, frame));

    EXPECT_EQ(0u, filesIn(dir));
}
