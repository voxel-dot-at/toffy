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

#include <memory>

#include <toffy/filter.hpp>
#include <toffy/filterfactory.hpp>

/**
 * @brief Minimal concrete Filter for exercising FilterBank mechanics.
 *
 * Ownership (P2-5): a Filter is a FilterPtr, and adding one to a bank shares
 * ownership of it. The bank no longer releases filters by asking the factory to
 * `delete` them by name, so a DummyFilter created with plain `new` and added to
 * a bank is destroyed correctly now -- but make() still goes through the
 * factory, because that is the path production takes and it is what registers
 * the type (see makeDummyFilter below).
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

/// Creates a DummyFilter through the factory and hands back shared ownership.
///
/// Registration happens here on purpose. FilterFactory::createFilter() looks
/// up unknown types in the creator map, and before P0-10 that lookup used
/// operator[], which inserted a null creator and then called it (findings
/// A18/A20), so an unregistered "dummy" segfaulted instead of failing cleanly.
/// Self-registering makes that unreachable from a test.
///
/// Returns shared_ptr<DummyFilter> rather than DummyFilter* (P2-5): the caller
/// owns the filter, hands it to a bank by sharing it, and can keep a reference
/// to check what happened to it afterwards.
inline std::shared_ptr<DummyFilter> makeDummyFilter()
{
    registerDummyFilter();
    return std::static_pointer_cast<DummyFilter>(
        toffy::FilterFactory::getInstance()->createFilter("dummy"));
}
