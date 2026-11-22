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

#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <boost/any.hpp>
#include <opencv2/core.hpp>

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

// Regression tests for cleanup/plan/p0-correctness.md P0-6 / findings A5 and A13.
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

// ---------------------------------------------------------------------------
// P2-11 — the Frame accessor API
//
// Three defects, all of them things the compiler could not previously tell you.
// ---------------------------------------------------------------------------

/** Can `optString` be called with a literal default?
 *
 *  It used to declare `std::string& dfault` -- a non-const reference -- while
 *  every sibling `opt*` took its default by value. So `optString("k", "n/a")`
 *  did not compile, and neither did a `const std::string`. A caller who wanted a
 *  fallback had to declare a mutable local first, which is not what an "optional
 *  get" means.
 */
template <typename T, typename = void>
struct OptStringAcceptsLiteral : std::false_type
{};

template <typename T>
struct OptStringAcceptsLiteral<
    T, std::void_t<decltype(std::declval<const T&>().optString(
           "k", "fallback"))>> : std::true_type
{};

static_assert(OptStringAcceptsLiteral<Frame>::value,
              "Frame::optString must accept a literal default, like optInt, "
              "optFloat and optMatPtr already do");

TEST(FrameIntegralTypes, LongRoundTripsAsLong)
{
    // `addData(long)` used to store the value under the `Int` tag while the
    // boost::any really held a long. getInt() does any_cast<int>, which checks
    // the real C++ type and not the tag -- so the tag was decorative and the
    // getter threw. The fix is to tag it honestly and give it a getter.
    Frame f;
    f.addData("l", 42L);
    EXPECT_EQ(Frame::Long, f.getDataType("l"))
        << "a long stored under the Int tag: the tag does not describe the value";
    EXPECT_EQ(42L, f.getLong("l"));
}

TEST(FrameIntegralTypes, UnsignedLongRoundTripsAsULong)
{
    Frame f;
    f.addData("ul", 42UL);
    EXPECT_EQ(Frame::ULong, f.getDataType("ul"));
    EXPECT_EQ(42UL, f.getULong("ul"));
}

// The real-world shape of the bug: modules/filters/src/tracking/tracker.cpp:265
// does `out.addData(_out_count, detObjs.size())`.
TEST(FrameIntegralTypes, SizeTypeRoundTripsAsULong)
{
    // size() returns size_t, which on LP64 is unsigned long, so this went through
    // the unsigned-long overload and landed under the Uint tag. Nothing in toffy
    // or either consumer reads it back today, which is the only reason this has
    // not arrived as a crash report.
    std::vector<int> objects(7);
    Frame f;
    f.addData("count", objects.size());
    EXPECT_EQ(Frame::ULong, f.getDataType("count"));
    EXPECT_EQ(7u, f.getULong("count"));
}

TEST(FrameIntegralTypes, IntAndLongAreNotInterchangeable)
{
    // Kept deliberately, and it passes before the fix too. The point is not that
    // the throw is nice -- it is that after P2-11 the tag tells the truth, so a
    // bad_any_cast here means the caller asked for the wrong type rather than
    // that the Frame mislabelled what it stored.
    Frame f;
    f.addData("l", 42L);
    EXPECT_THROW(f.getInt("l"), boost::bad_any_cast);

    Frame g;
    g.addData("i", 42);
    EXPECT_THROW(g.getLong("i"), boost::bad_any_cast);
}

TEST(FrameAccessors, GetOrInsertMatPtrIsSpelledProperly)
{
    Frame f;
    const cv::Size sz(8, 6);
    toffy::matPtr m = f.getOrInsertMatPtr("m", sz, CV_8UC1);
    ASSERT_NE(nullptr, m);
    EXPECT_EQ(8, m->cols);
    EXPECT_EQ(6, m->rows);

    // The "or insert" half is the whole contract: a second call returns the same
    // buffer, so a filter can lazily obtain an output image.
    toffy::matPtr again = f.getOrInsertMatPtr("m", sz, CV_8UC1);
    EXPECT_EQ(m->data, again->data);
}
