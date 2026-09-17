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
