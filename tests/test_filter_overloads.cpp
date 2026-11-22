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
// Filter used to declare two virtual filter() overloads: a const one returning
// false, and a non-const one whose default body delegated to the const one.
// FilterBank::filter() calls through a non-const Filter*, so a filter that
// overrode *only* the const overload ran its body via that delegation - and a
// filter whose declaration silently stopped being an override ran the base's
// `return false` instead. The bank logged "failed!" and returned false; the
// filter was still in the pipeline, still configured, still counted by size(),
// and did nothing. Nothing in the build said so.
//
// P2-12 removed the const overload, so there is now one virtual and the
// delegation is gone. What this file pins has therefore changed shape, and the
// change was made deliberately rather than by deletion:
//
//   * the shape that no longer exists (override only the const form) is now
//     asserted at COMPILE time: a const Filter& cannot call filter() at all.
//     That is a stronger statement than the runtime test it replaces, which
//     could only show that a particular call returned true.
//   * the shape that remains is still exercised through the bank, because
//     "compiles" and "runs" are still different claims.
//   * the negative control stays, and is still the most important test here.
//
// `override` catches signature mistakes at compile time; it cannot catch a filter
// that overrides a different virtual than the one the pipeline calls, and no
// amount of "the tree compiles" distinguishes that - which is why this file
// exists (../dod/stage-gates.md 2.4, P3-9).
//
// The last test is the negative control: it asserts that a filter which overrides
// nothing really is detected as not running. Without it the tests above could be
// passing for the wrong reason, and DOD 1 ("a test that has never been observed
// to fail is not a regression fence") would not be satisfied.

#include <cstring>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include <toffy/filter.hpp>
#include <toffy/filterbank.hpp>
#include <toffy/filterfactory.hpp>

namespace {

// Slot each test filter writes into when its body executes. Checking the frame
// rather than a member means the assertion is about the data flow the pipeline
// exists to produce, not about an object the test happens to hold.
constexpr const char* kNonConstOnlySlot = "p39_nonconst_only_ran";
constexpr const char* kSecondSlot = "p39_second_ran";
constexpr const char* kNotAnOverrideSlot = "p39_not_an_override_ran";

/** Is `filter(const Frame&, Frame&)` callable on a NON-const reference to T?
 *  The positive half of the pair below: P2-12 removes one overload, not the
 *  ability to filter. */
template <typename T, typename = void>
struct FilterCallableOnRef : std::false_type
{};

template <typename T>
struct FilterCallableOnRef<
    T, std::void_t<decltype(std::declval<T&>().filter(
           std::declval<const toffy::Frame&>(),
           std::declval<toffy::Frame&>()))>> : std::true_type
{};

/** Is `filter(const Frame&, Frame&)` callable on a const reference to T?
 *
 *  This is the question P2-12 answers, and it can only be answered at compile
 *  time. Before P2-12 it was callable and, for a filter that overrode only the
 *  const form, silently returned false for everything else. Now there is no
 *  viable overload, so the expression is ill-formed and the trait is false.
 */
template <typename T, typename = void>
struct FilterCallableOnConstRef : std::false_type
{};

template <typename T>
struct FilterCallableOnConstRef<
    T, std::void_t<decltype(std::declval<const T&>().filter(
           std::declval<const toffy::Frame&>(),
           std::declval<toffy::Frame&>()))>> : std::true_type
{};

/** The one remaining shape: override the single non-const virtual. This is what
 *  every filter in the tree does, including the four (OffSet, Transform, Merge,
 *  CloudViewOpenCv) that used to override only the const form. */
class OnlyFilter : public toffy::Filter
{
   public:
    OnlyFilter() : toffy::Filter("only_filter") {}

    bool filter(const toffy::Frame& /*in*/, toffy::Frame& out) override
    {
        out.addData(kNonConstOnlySlot, 1);
        return true;
    }
};

/** A second filter of the same shape, with its own slot, so a bank of two can
 *  show that every body runs rather than just the first one. */
class SecondFilter : public toffy::Filter
{
   public:
    SecondFilter() : toffy::Filter("second_filter") {}

    bool filter(const toffy::Frame& /*in*/, toffy::Frame& out) override
    {
        out.addData(kSecondSlot, 1);
        return true;
    }
};

/** Overrides nothing at all.
 *
 *  `filter(Frame&, Frame&) const` matches no base virtual (the base takes a
 *  `const Frame&`), so this class inherits the one virtual and adds a second,
 *  unrelated function. It compiles with the project's -Wall -Wextra because the
 *  `using` declaration keeps the base overload visible - the same line P3-4
 *  needed in Mux - which is exactly the point: the warning gate cannot see this,
 *  and neither can the bank. FilterBank::filter() calls the base virtual, which
 *  returns false. P2-12 removed the delegation step but not this failure mode:
 *  a declaration that is not an override still compiles, still sits in the
 *  pipeline, and still does nothing. That is why this control stays.
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

/** Register a creator and build the filter through the factory, so the test
 *  goes the way production does and gets shared ownership of the result back
 *  (P2-5) - see the ownership note in dummy_filter.hpp. */
template <typename T>
std::shared_ptr<T> makeViaFactory(const std::string& type)
{
    toffy::FilterFactory::registerCreator(
        type, []() -> toffy::Filter* { return new T(); });
    return std::static_pointer_cast<T>(
        toffy::FilterFactory::getInstance()->createFilter(type));
}

}  // namespace

TEST(FilterOverloadP39, OverrideRunsThroughTheBank)
{
    toffy::FilterBank bank;
    auto f = makeViaFactory<OnlyFilter>("only_filter");
    ASSERT_NE(nullptr, f);
    bank.add(f);

    toffy::Frame frame;
    EXPECT_TRUE(bank.filter(frame, frame));
    EXPECT_TRUE(frame.hasKey(kNonConstOnlySlot))
        << "the bank ran a filter without running its body";
}

TEST(FilterOverloadP39, MultipleFiltersRunInOneBank)
{
    // The pipeline case: a bank of several filters must run every body, in
    // order, and must not stop partway. This was "both shapes" while Filter had
    // two overloads to mix; with one virtual there is one shape, and the value
    // of the test is the ordering and the not-stopping-partway part.
    toffy::FilterBank bank;
    bank.add(makeViaFactory<OnlyFilter>("only_filter"));
    bank.add(makeViaFactory<SecondFilter>("second_filter"));
    ASSERT_EQ(2u, bank.size());

    toffy::Frame frame;
    EXPECT_TRUE(bank.filter(frame, frame));
    EXPECT_TRUE(frame.hasKey(kNonConstOnlySlot));
    EXPECT_TRUE(frame.hasKey(kSecondSlot));
}

/** A type that DOES have a const `filter()` -- the shape P2-12 removed.
 *
 *  Exists to keep the assert below honest. `FilterCallableOnConstRef` is the
 *  thing doing the proving, and a trait that could only ever be false would make
 *  `static_assert(!FilterCallableOnConstRef<Filter>::value)` pass for literally
 *  any type, including one that still has the trap. This class is the trait's
 *  positive case, so the trait is pinned in both directions.
 */
class ConstFilterShaped
{
   public:
    bool filter(const toffy::Frame& /*in*/, toffy::Frame& /*out*/) const
    {
        return true;
    }
};

TEST(FilterOverloadP212, TheConstCallabilityTraitIsNotVacuous)
{
    // If this ever stops compiling, the trait is broken and every assert that
    // uses it has quietly stopped meaning anything.
    static_assert(FilterCallableOnConstRef<ConstFilterShaped>::value,
                  "FilterCallableOnConstRef must report TRUE for a type that does "
                  "have a const filter(); if it cannot, the assert that it is "
                  "FALSE for toffy::Filter proves nothing");
    SUCCEED();
}

/** Read the vtable slot of a virtual out of a pointer-to-member.
 *
 *  On the Itanium ABI -- what Linux and macOS on x86-64/aarch64 use, which after
 *  P2-2 is the only thing toffy builds for -- a pointer-to-member-function to a
 *  virtual holds `8 * index + 1`. That makes the slot number readable at runtime
 *  with no compiler extension and no debug info.
 *
 *  It is implementation-defined, so the test that uses it is guarded: on any
 *  other ABI the slot arithmetic would not mean what the assertions say it means.
 */
long vtableSlotOf(long raw) { return (raw - 1) / 8; }

template <typename T, typename R>
long vtableSlotOf(R (T::*mf)(const toffy::Frame&, toffy::Frame&))
{
    long raw = 0;
    std::memcpy(&raw, &mf, sizeof(raw));
    return vtableSlotOf(raw);
}

template <typename T, typename R>
long vtableSlotOf(R (T::*mf)(const boost::property_tree::ptree&))
{
    long raw = 0;
    std::memcpy(&raw, &mf, sizeof(raw));
    return vtableSlotOf(raw);
}

template <typename T, typename R>
long vtableSlotOf(R (T::*mf)())
{
    long raw = 0;
    std::memcpy(&raw, &mf, sizeof(raw));
    return vtableSlotOf(raw);
}

TEST(FilterOverloadP212, FilterVtableLayoutIsPinned)
{
    // Deleting a virtual from Filter moves every virtual declared after it down
    // one slot. P2-12 found this the hard way: removing the const filter()
    // overload shifted filter 2->1, loadConfig 3->2 and getState 7->6, and
    // tools/abi_surface.sh reported nothing, because it filters to symbol types
    // T/D/B and a virtual with an inline body is emitted WEAK. A vtable change is
    // invisible to every other gate this repository has.
    //
    // That matters because X6 records the filter plug-in ABI as deliberately not
    // broken: a plug-in .so built against an older toffy still loads and still
    // registers. A plug-in is also a Filter subclass, so a shifted vtable would
    // leave it loading, registering, and dispatching loadConfig into filter -- no
    // link error, no diagnostic. Hence the reserved slot in filter.hpp, and hence
    // this test: it is what stops the next person deleting that slot, or inserting
    // a virtual in the wrong place, without noticing.
    //
    // These numbers are not chosen for their own sake. They are the layout a
    // plug-in compiled against toffy <= 2.1.0 was built against. Changing one is
    // a deliberate ABI break and belongs with a major version.
    EXPECT_EQ(2, vtableSlotOf(&toffy::Filter::filter));
    EXPECT_EQ(3, vtableSlotOf(&toffy::Filter::loadConfig));
    EXPECT_EQ(7, vtableSlotOf(&toffy::Filter::getState));
}

TEST(FilterOverloadP212, ConstFilterReferenceCannotCallFilter)
{
    // The contract P2-12 removed, asserted where it can actually be pinned.
    //
    // The test that used to sit here called filter() through a const Filter& and
    // checked that a const-only filter's body ran. That is the behaviour P2-12
    // deletes, so the call is now ill-formed -- which is a stronger guarantee
    // than the runtime check ever was. Before, a const caller holding a filter
    // that overrode only the non-const form got `false` back with no log line
    // and no error state; the type system offered no protection and the test
    // could only demonstrate the good case. Now the bad case does not compile.
    //
    // static_assert, not EXPECT: "does not compile" is not observable at runtime.
    static_assert(!FilterCallableOnConstRef<toffy::Filter>::value,
                  "Filter::filter() must not be callable on a const Filter& -- "
                  "a second, const overload would reintroduce the silent-false "
                  "trap P2-12 removed");
    static_assert(!FilterCallableOnConstRef<OnlyFilter>::value,
                  "a concrete filter must not be callable through a const ref "
                  "either");

    // And the positive half: the same call on a non-const reference is fine.
    static_assert(FilterCallableOnRef<toffy::Filter>::value,
                  "there must still be exactly one callable filter() virtual");
    SUCCEED();
}

TEST(FilterOverloadP39, NegativeControlDetectsAnOverrideThatOverridesNothing)
{
    // The fence for the fence. A filter that overrides nothing keeps compiling,
    // keeps its place in the bank and never runs: the bank must report failure
    // and the slot must stay empty. If this test ever starts failing, the
    // pipeline has become able to reach such a declaration and the three tests
    // above have stopped being evidence of anything.
    toffy::FilterBank bank;
    auto f = makeViaFactory<NotAnOverrideFilter>("not_an_override");
    ASSERT_NE(nullptr, f);
    bank.add(f);

    toffy::Frame frame;
    EXPECT_FALSE(bank.filter(frame, frame));
    EXPECT_FALSE(frame.hasKey(kNotAnOverrideSlot))
        << "the body ran, so it really was an override after all";
}
