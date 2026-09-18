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

// Unit tests for toffy::Frame.

#include <gtest/gtest.h>

#include <toffy/frame.hpp>

using toffy::Frame;

TEST(FrameSmoke, NewFrameIsEmpty)
{
    Frame f;
    EXPECT_FALSE(f.hasKey("anything"));
}

TEST(FrameSmoke, AddAndRetrieveInt)
{
    Frame f;
    f.addData("answer", 42);

    ASSERT_TRUE(f.hasKey("answer"));
    EXPECT_EQ(42, f.getInt("answer"));
}

// Regression tests for CLEANUP_PLAN P0-6 / findings A5 and A13.
//
// Frame keeps three parallel maps: data, meta (the SlotDataType tag) and desc.
// Only `data` was copied, assigned, cleared and erased from, so hasKey() and
// getDataType() disagreed about whether a slot existed.

namespace {
// Adds a slot carrying a description, so the `desc` map is populated too.
void addDescribedSlot(Frame& f)
{
    f.addData("depth", 1.5, Frame::Double, "depth in mm");
}
}  // namespace

TEST(FrameMetadata, CopyConstructorPreservesMetaAndDesc)
{
    Frame a;
    addDescribedSlot(a);

    Frame b(a);

    ASSERT_TRUE(b.hasKey("depth"));
    EXPECT_EQ(Frame::Double, b.getDataType("depth"));
    EXPECT_EQ("depth in mm", b.getDescription("depth"));
}

TEST(FrameMetadata, AssignmentPreservesMetaAndDesc)
{
    Frame a;
    addDescribedSlot(a);

    Frame b;
    b = a;

    ASSERT_TRUE(b.hasKey("depth"));
    EXPECT_EQ(Frame::Double, b.getDataType("depth"))
        << "operator= copied only `data`, so every type tag was lost";
    EXPECT_EQ("depth in mm", b.getDescription("depth"));
}

TEST(FrameMetadata, RemoveDataClearsAllThreeMaps)
{
    Frame f;
    addDescribedSlot(f);

    ASSERT_TRUE(f.removeData("depth"));

    EXPECT_FALSE(f.hasKey("depth"));
    EXPECT_EQ(Frame::NotFound, f.getDataType("depth"))
        << "getDataType() still reports a type for a key that is gone";
    EXPECT_TRUE(f.getDescription("depth").empty());
}

TEST(FrameMetadata, ClearDataClearsAllThreeMaps)
{
    Frame f;
    addDescribedSlot(f);

    f.clearData();

    EXPECT_FALSE(f.hasKey("depth"));
    EXPECT_EQ(Frame::NotFound, f.getDataType("depth"));
    EXPECT_TRUE(f.getDescription("depth").empty());
}
