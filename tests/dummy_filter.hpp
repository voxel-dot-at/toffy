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
#pragma once

#include <toffy/filter.hpp>
#include <toffy/filterfactory.hpp>

/**
 * @brief Minimal concrete Filter for exercising FilterBank mechanics.
 *
 * Instances must be created through makeDummyFilter(), i.e. via the
 * FilterFactory, so that ownership matches production: FilterBank::clearBank()
 * and FilterBank::remove() release filters by calling
 * FilterFactory::deleteFilter(name), which only knows about filters the
 * factory created. A DummyFilter built with plain new is therefore never
 * released -- that leak is what P2-5 removes, and it would make the ASan gate
 * in DOD.md 2.1 report the harness instead of the code under test.
 */
class DummyFilter : public toffy::Filter
{
   public:
    DummyFilter() : toffy::Filter("dummy"), wasCalled(false)
    {
        // Not strictly needed, but keeps the helper independent of the
        // uninitialised-member bug fixed separately in P0-5.
        bank(nullptr);
    }

    bool filter(const toffy::Frame& /*in*/, toffy::Frame& /*out*/) override
    {
        wasCalled = true;
        return true;
    }

    bool wasCalled;
};

/// Registers the "dummy" creator. Calling it more than once just overwrites
/// the same entry, which is harmless for tests.
inline void registerDummyFilter()
{
    toffy::FilterFactory::registerCreator(
        "dummy", []() -> toffy::Filter* { return new DummyFilter(); });
}

/// Creates a DummyFilter through the factory so the bank can release it.
///
/// Registration happens here on purpose. FilterFactory::createFilter() looks
/// up unknown types with operator[] on the creator map, which inserts a null
/// creator and then calls it (findings A18/A20), so an unregistered "dummy"
/// segfaults instead of failing cleanly. Self-registering makes that
/// unreachable from a test.
inline DummyFilter* makeDummyFilter()
{
    registerDummyFilter();
    return static_cast<DummyFilter*>(
        toffy::FilterFactory::getInstance()->createFilter("dummy"));
}
