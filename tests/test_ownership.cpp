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

// Ownership of a Filter (P2-5).
//
// The arrangement before this item: FilterBank::_pipe held raw Filter*, the
// FilterFactory registry held raw Filter* to the same objects, and the teardown
// went through the factory *by name* -- ~FilterBank::clearBank() called
// ff->deleteFilter(_pipe[i]->name()) and FilterFactory::deleteFilter() did
// `delete it->second`. Two borrowers and no owner, with a mutable string as the
// key. The consequences, all of them reachable through the public API and each
// pinned by a test below:
//
//   * a filter whose name had been changed was not found by that lookup, so it
//     was never released (leak);
//   * a filter held by two banks was destroyed by whichever bank went first,
//     leaving the other one holding freed memory (use-after-free on the next
//     frame, and the ASan job's subject);
//   * FilterBank::remove() destroyed the filter it removed, so a caller that
//     still pointed at one was reading freed memory on the next statement;
//   * the registry kept every filter it had ever created alive forever, and
//     findFilter() kept reporting names whose objects nothing needed.
//
// Now: the bank owns what it holds (a vector of FilterPtr), the factory indexes
// with weak_ptr, and nothing in the tree calls delete on a Filter.
//
// A note on how these tests are written, because it is deliberate. Every filter
// comes from the factory into an `auto`, and no FilterPtr is spelled out in the
// test bodies. That is what lets the same file compile against the old
// raw-pointer API and the new one, which is what makes it a fence rather than a
// description: the pre-fix behaviour was measured by building this file against
// the parent commit, where 8 of the 9 tests fail — `BankReleasesWhatItHolds`
// passes there too, and stays as the pin that the release *does* happen, just
// not through the bank. Five of the eight produce a sanitizer report:
//
//   FilterInTwoBanksSurvivesTheFirstOne      heap-use-after-free
//   RemoveDoesNotDestroyTheFilterItRemoved   heap-use-after-free
//   EmptyFilterIsNotPipelined                SEGV in FilterBank::clearBank()
//   InsertRejectsAnOutOfRangePosition        heap-buffer-overflow in _pipe
//   PlayerSessionReleasesEverythingOnShutdown  direct leak of 184 bytes
//
// Run them one at a time on the parent commit: the second test aborts the
// process mid-suite, which is itself the point but makes a single run look like
// two failures. A test that only compiles after the fix proves nothing about
// what the fix changed.

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>

#include <toffy/filter.hpp>
#include <toffy/filterbank.hpp>
#include <toffy/filterfactory.hpp>
#include <toffy/player.hpp>

using toffy::FilterFactory;

namespace {

/** How many counted objects exist right now. The whole point of the file. */
std::atomic<int>& live()
{
    static std::atomic<int> n(0);
    return n;
}

/** How many times a counted filter's body ran. */
std::atomic<int>& calls()
{
    static std::atomic<int> n(0);
    return n;
}

/**
 * A Filter that counts itself in and out.
 *
 * "Was this filter destroyed, and exactly how many times" is otherwise invisible
 * from outside, and both the leak and the double free are questions about a
 * destructor running at the wrong moment.
 *
 * Both constructors pass a type to the base: Filter()'s default form leaves
 * `_name` and `_id` *empty* (it only sets `_type` to the
 * "Filter.thisShouldNotHappen!" marker), and an empty name makes the name-keyed
 * assertions below pass or fail for reasons that have nothing to do with
 * ownership -- findFilter("") and remove("") both do something surprising with
 * an empty string. Every real filter goes through Filter(type, counter).
 */
class CountedFilter : public toffy::Filter
{
   public:
    CountedFilter() : toffy::Filter("counted") { ++live(); }
    virtual ~CountedFilter() { --live(); }

    // Returns true, so that a bank full of these actually runs to the end of
    // the pipeline: FilterBank::filter() stops at the first child that returns
    // false, and the Player session below would otherwise only ever exercise
    // the first filter in each bank.
    //
    // The non-const overload, with `override` -- the shape P3-3/P3-9 exist to
    // keep honest.
    bool filter(const toffy::Frame& /*in*/, toffy::Frame& /*out*/) override
    {
        ++calls();
        return true;
    }
};

/** The same, for a bank: a bank is a Filter, and it is owned the same way. */
class CountedBank : public toffy::FilterBank
{
   public:
    CountedBank() : toffy::FilterBank("countedBank", 0) { ++live(); }
    virtual ~CountedBank() { --live(); }
};

/** Resets the counters and registers both types. Call at the top of a test. */
void resetCounted()
{
    live().store(0);
    calls().store(0);
    FilterFactory::registerCreator(
        "counted", []() -> toffy::Filter* { return new CountedFilter(); });
    FilterFactory::registerCreator(
        "countedBank", []() -> toffy::Filter* { return new CountedBank(); });
}

int count() { return live().load(); }

}  // namespace

// The bank is what keeps its filters alive: add one and let the bank go and the
// filter is gone with it. Pre-fix this one passes for the wrong reason -- the
// release happened through the factory's registry rather than through the bank,
// and only while the name still matched. The next test is the one that hurts.
TEST(FilterOwnership, BankReleasesWhatItHolds)
{
    resetCounted();
    {
        toffy::FilterBank bank;
        bank.add(FilterFactory::getInstance()->createFilter("counted"));
        ASSERT_EQ(1, count());
    }
    EXPECT_EQ(0, count()) << "the bank went away and left its filter behind";
}

// The fence for the leak. Names are mutable -- Filter::name() is public and
// loadConfig() writes it -- and the old teardown looked the filter up in the
// factory by name while createFilter() had registered it by id. Rename it and
// the lookup finds nothing, so nothing is released.
TEST(FilterOwnership, RenamedFilterIsReleasedWithTheBank)
{
    resetCounted();
    {
        toffy::FilterBank bank;
        auto f = FilterFactory::getInstance()->createFilter("counted");
        bank.add(f);
        f->name("renamed");
        ASSERT_EQ(1, count());
    }
    EXPECT_EQ(0, count())
        << "clearBank() looked the filter up by its new name, the registry "
           "knew it by its id, and the lookup found nothing";
}

// The fence for the use-after-free. A filter reachable from two banks used to be
// destroyed by whichever bank was torn down first while the other one went on
// holding the pointer.
TEST(FilterOwnership, FilterInTwoBanksSurvivesTheFirstOne)
{
    resetCounted();
    toffy::FilterBank a, b;
    auto f = FilterFactory::getInstance()->createFilter("counted");
    a.add(f);
    b.add(f);
    ASSERT_EQ(1, count());

    a.clearBank();

    EXPECT_EQ(1, count())
        << "clearing one bank destroyed a filter the other one still holds";
    ASSERT_EQ(1u, b.size());
    // Reading through the surviving bank is the point: pre-fix this reads freed
    // memory, which the sanitizer job reports as heap-use-after-free.
    EXPECT_FALSE(b.getFilter(0)->name().empty());

    b.clearBank();
    EXPECT_EQ(1, count()) << "the second bank destroyed it a second time";

    // And once the last owner lets go it is gone -- exactly once in total.
    f = nullptr;
    EXPECT_EQ(0, count());
}

// The fence for remove(). It used to erase the entry and then ask the factory to
// delete the filter, so a caller holding the filter it had just removed was
// holding freed memory.
TEST(FilterOwnership, RemoveDoesNotDestroyTheFilterItRemoved)
{
    resetCounted();
    toffy::FilterBank bank;
    auto f = FilterFactory::getInstance()->createFilter("counted");
    const std::string name = f->name();
    bank.add(f);

    ASSERT_EQ(1, bank.remove(name));
    ASSERT_EQ(0u, bank.size());

    EXPECT_EQ(1, count())
        << "remove() destroyed a filter the caller still holds";
    EXPECT_EQ(name, f->name())
        << "reading the removed filter is reading freed memory";
}

// An empty filter is refused instead of being pipelined: a null in _pipe is a
// crash several frames later, far from the call that put it there. `unknown` is
// what the factory hands back for a type it cannot resolve, so this is the shape
// the failure actually arrives in -- a config file with a typo in it.
TEST(FilterOwnership, EmptyFilterIsNotPipelined)
{
    resetCounted();
    toffy::FilterBank bank;
    auto unknown = FilterFactory::getInstance()->createFilter("no_such_type");

    bank.add(unknown);
    EXPECT_EQ(0u, bank.size()) << "an empty filter entered the pipeline";

    bank.insert(0, unknown);
    EXPECT_EQ(0u, bank.size());
}

// insert() had no bounds check at all: `_pipe.insert(_pipe.begin() + pos, f)`.
TEST(FilterOwnership, InsertRejectsAnOutOfRangePosition)
{
    resetCounted();
    toffy::FilterBank bank;
    bank.add(FilterFactory::getInstance()->createFilter("counted"));

    EXPECT_THROW(bank.insert(5, FilterFactory::getInstance()
                                   ->createFilter("counted")),
                 std::out_of_range);
    EXPECT_THROW(bank.insert(-1, FilterFactory::getInstance()
                                     ->createFilter("counted")),
                 std::out_of_range);
    EXPECT_EQ(1, count())
        << "a rejected insert must not keep the filter it refused";

    // pos == size() appends, which is a legal position for vector::insert.
    bank.insert(1, FilterFactory::getInstance()->createFilter("counted"));
    ASSERT_EQ(2u, bank.size());
    EXPECT_EQ(2, count());
}

// The factory's index is weak, so it cannot keep a filter alive: a filter
// created and dropped in the same scope is gone, and the index says so rather
// than advertising a name that resolves to nothing.
TEST(FilterOwnership, FactoryIndexDoesNotKeepFiltersAlive)
{
    resetCounted();
    std::string name;
    {
        auto f = FilterFactory::getInstance()->createFilter("counted");
        ASSERT_NE(nullptr, f);
        name = f->name();
        ASSERT_TRUE(FilterFactory::getInstance()->findFilter(name));
        EXPECT_EQ(1, count());
    }
    EXPECT_EQ(0, count())
        << "the registry kept the only reference to a filter nobody asked for";
    EXPECT_FALSE(FilterFactory::getInstance()->findFilter(name));
}

// A sub-bank goes with its parent, including one that was renamed and one that
// has children of its own. The children are built through the config path,
// because that is how sub-banks get filled in production and it is the path the
// old teardown was wrong on. Pre-fix nothing here is released: the outer bank
// asks the factory for "renamed-inner", does not find it, and the whole subtree
// leaks.
TEST(FilterOwnership, NestedBankGoesWithItsParent)
{
    resetCounted();
    {
        toffy::FilterBank outer;
        auto inner = FilterFactory::getInstance()->createFilter("countedBank");
        boost::property_tree::ptree pt;
        // add(), not add_child(): this is the *data* form, which creates the
        // <toffy><counted/></toffy> path that FilterBank::loadConfig() walks.
        pt.add("toffy.counted", "");
        inner->loadConfig(pt);
        ASSERT_EQ(2, count()) << "the config path did not fill the sub-bank";

        inner->name("renamed-inner");
        outer.add(inner);
    }
    EXPECT_EQ(0, count())
        << "the nested bank, or something inside it, survived the parent";
}

// The DOD 2.3 gate for this item is "a full session, including shutdown, with
// zero leaks and zero double-frees". A test does that better than a hand-run of
// toffyRunner, because it is the same assertion on every push and it runs inside
// the ASan job: the tree is built from a real XML file through the real Player
// path — Player::loadConfig → Controller's root FilterBank → handleConfigItem →
// the factory — frames are run through it, and the Player is destroyed.
//
// What makes this a fence and not a pin is the `<name>` on the first filter.
// Filter::updateConfig() honours it, and the old teardown released a bank's
// children by asking the factory for them *by name* while createFilter() had
// registered them by id — so the renamed filter was not found, was never
// deleted, and everything it owned survived the Player. Pre-fix this reports
// exactly one counted filter still alive once the Player is gone — the renamed
// one. The other two are found by name and released, which is why the old
// teardown looked correct most of the time.
TEST(FilterOwnership, PlayerSessionReleasesEverythingOnShutdown)
{
    resetCounted();

    const std::string dir = ::testing::TempDir() + "/ownership_session";
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir);
    const std::string file = dir + "/config.xml";
    {
        std::ofstream out(file);
        // <cond> is in the factory's built-in chain and its own loadConfig()
        // walks a <filterBank> child, which is what puts the last two filters
        // inside a nested bank instead of at the top level.
        out << "<?xml version=\"1.0\"?>\n<toffy>\n"
            << "  <counted><name>renamed</name></counted>\n"
            << "  <cond>\n    <name>theCondition</name>\n"
            << "    <filterBank>\n      <counted/>\n      <countedBank/>\n"
            << "    </filterBank>\n  </cond>\n</toffy>\n";
    }

    {
        toffy::Player player;
        ASSERT_GT(player.loadConfig(file), 0)
            << "the config did not load, so the rest of this test proves nothing";
        // Three counted objects: the renamed top-level filter, and the counted
        // filter and the bank inside <cond>'s <filterBank>. The <cond> itself is
        // a real Cond, which is not one of the counted types — it is there
        // because it is the built-in filter that walks a <filterBank> child and
        // is therefore what makes the tree two levels deep.
        ASSERT_EQ(3, count()) << "the config path did not build the whole tree";

        player.runOnce();
        player.runOnce();
        // The frames have to have reached the filters, or "nothing leaked" would
        // also be satisfied by a session that never ran anything.
        EXPECT_GE(calls(), 2) << "no counted filter ran during the session";
    }

    EXPECT_EQ(0, count())
        << "a filter outlived the Player that owned it";
}
