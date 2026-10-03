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

// P3-9: a test that a filter's body actually ran.
//
// Filter declares two virtual filter() overloads (filter.hpp): a const one that
// returns false, and a non-const one whose default body delegates to the const
// one. FilterBank::filter() calls through a non-const Filter*, so a filter that
// overrides *only* the const overload runs its body via that delegation - and a
// filter whose declaration silently stops being an override runs the base's
// `return false` instead. The bank then logs "failed!" and returns false; the
// filter is still in the pipeline, still configured, still counted by size(),
// and does nothing. Nothing in the build says so.
//
// That is the failure P2-12 (delete the const overload, keep one virtual) can
// cause, and the reason P3-3 put `override` on the four const-only filters in
// modules/filters first. `override` catches the *signature* case at compile
// time; it cannot catch a filter that overrides a different virtual than the one
// the pipeline calls, and no existing test distinguished "compiles" from "runs" -
// which is why this file exists (../dod/stage-gates.md 2.4, P3-9).
//
// The last test is the negative control: it asserts that a filter which overrides
// nothing really is detected as not running. Without it the three tests above
// could be passing for the wrong reason, and DOD 1 ("a test that has never been
// observed to fail is not a regression fence") would not be satisfied.

#include <string>

#include <gtest/gtest.h>

#include <toffy/filter.hpp>
#include <toffy/filterbank.hpp>
#include <toffy/filterfactory.hpp>

namespace {

// Slot each test filter writes into when its body executes. Checking the frame
// rather than a member means the assertion is about the data flow the pipeline
// exists to produce, not about an object the test happens to hold.
constexpr const char* kConstOnlySlot = "p39_const_only_ran";
constexpr const char* kNonConstOnlySlot = "p39_nonconst_only_ran";
constexpr const char* kNotAnOverrideSlot = "p39_not_an_override_ran";

/** Overrides only the const overload - OffSet, Transform, Merge and
 *  CloudViewOpenCv in modules/filters are all of this shape. */
class ConstOnlyFilter : public toffy::Filter
{
   public:
    ConstOnlyFilter() : toffy::Filter("const_only") {}

    bool filter(const toffy::Frame& /*in*/, toffy::Frame& out) const override
    {
        out.addData(kConstOnlySlot, 1);
        return true;
    }
};

/** Overrides only the non-const overload - the shape DummyFilter and most of
 *  the tree use. */
class NonConstOnlyFilter : public toffy::Filter
{
   public:
    NonConstOnlyFilter() : toffy::Filter("nonconst_only") {}

    bool filter(const toffy::Frame& /*in*/, toffy::Frame& out) override
    {
        out.addData(kNonConstOnlySlot, 1);
        return true;
    }
};

/** Overrides nothing at all.
 *
 *  `filter(Frame&, Frame&) const` matches neither base virtual (the base takes
 *  a `const Frame&`), so this class inherits both and adds a third. It compiles
 *  with the project's -Wall -Wextra because the `using` declaration keeps the
 *  base overloads visible - the same line P3-4 needed in Mux - which is exactly
 *  the point: the warning gate cannot see this, and neither can the bank.
 *  FilterBank::filter() passes a const Frame&, so the base's non-const virtual
 *  is called, delegates to the base's const virtual, and returns false.
 */
class NotAnOverrideFilter : public toffy::Filter
{
   public:
    NotAnOverrideFilter() : toffy::Filter("not_an_override") {}

    using Filter::filter;

    bool filter(toffy::Frame& /*in*/, toffy::Frame& out) const
    {
        out.addData(kNotAnOverrideSlot, 1);
        return true;
    }
};

/** Register a creator and build the filter through the factory, so that
 *  FilterBank::clearBank() can release it - see the ownership note in
 *  dummy_filter.hpp. Without this the bank leaks the filter and the ASan CI job
 *  reports the harness instead of the code under test. */
template <typename T>
T* makeViaFactory(const std::string& type)
{
    toffy::FilterFactory::registerCreator(
        type, []() -> toffy::Filter* { return new T(); });
    return static_cast<T*>(
        toffy::FilterFactory::getInstance()->createFilter(type));
}

}  // namespace

TEST(FilterOverloadP39, ConstOnlyOverrideRunsThroughTheBank)
{
    toffy::FilterBank bank;
    ConstOnlyFilter* f = makeViaFactory<ConstOnlyFilter>("const_only");
    ASSERT_NE(nullptr, f);
    bank.add(f);

    toffy::Frame frame;
    EXPECT_TRUE(bank.filter(frame, frame));
    EXPECT_TRUE(frame.hasKey(kConstOnlySlot))
        << "the bank ran a const-only filter without running its body";
}

TEST(FilterOverloadP39, NonConstOnlyOverrideRunsThroughTheBank)
{
    toffy::FilterBank bank;
    NonConstOnlyFilter* f = makeViaFactory<NonConstOnlyFilter>("nonconst_only");
    ASSERT_NE(nullptr, f);
    bank.add(f);

    toffy::Frame frame;
    EXPECT_TRUE(bank.filter(frame, frame));
    EXPECT_TRUE(frame.hasKey(kNonConstOnlySlot))
        << "the bank ran a non-const-only filter without running its body";
}

TEST(FilterOverloadP39, BothShapesRunInOneBank)
{
    // The pipeline case: a bank mixing the two override shapes must run both
    // bodies, in order, and must not stop at the const-only one.
    toffy::FilterBank bank;
    bank.add(makeViaFactory<ConstOnlyFilter>("const_only"));
    bank.add(makeViaFactory<NonConstOnlyFilter>("nonconst_only"));
    ASSERT_EQ(2u, bank.size());

    toffy::Frame frame;
    EXPECT_TRUE(bank.filter(frame, frame));
    EXPECT_TRUE(frame.hasKey(kConstOnlySlot));
    EXPECT_TRUE(frame.hasKey(kNonConstOnlySlot));
}

TEST(FilterOverloadP39, ConstReferenceStillReachesTheConstOverride)
{
    // Documents the contract P2-12 proposes to remove: a const Filter& gets the
    // const overload, and for a const-only filter that IS the body. When P2-12
    // deletes the base's const overload this test stops compiling - on purpose.
    // It is the compile-time notice that this behaviour changed.
    ConstOnlyFilter f;
    const toffy::Filter& constF = f;

    toffy::Frame frame;
    EXPECT_TRUE(constF.filter(frame, frame));
    EXPECT_TRUE(frame.hasKey(kConstOnlySlot));
}

TEST(FilterOverloadP39, NegativeControlDetectsAnOverrideThatOverridesNothing)
{
    // The fence for the fence. A filter that overrides nothing keeps compiling,
    // keeps its place in the bank and never runs: the bank must report failure
    // and the slot must stay empty. If this test ever starts failing, the
    // pipeline has become able to reach such a declaration and the three tests
    // above have stopped being evidence of anything.
    toffy::FilterBank bank;
    NotAnOverrideFilter* f =
        makeViaFactory<NotAnOverrideFilter>("not_an_override");
    ASSERT_NE(nullptr, f);
    bank.add(f);

    toffy::Frame frame;
    EXPECT_FALSE(bank.filter(frame, frame));
    EXPECT_FALSE(frame.hasKey(kNotAnOverrideSlot))
        << "the body ran, so it really was an override after all";
}
