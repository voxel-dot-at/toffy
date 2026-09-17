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

#include <string>

#include <toffy/filter.hpp>

/**
 * @brief Minimal concrete Filter for exercising FilterBank mechanics.
 *
 * Deliberately independent of the FilterFactory: bank tests must be able to
 * populate a bank without dragging in every built-in filter type. Note that
 * this means the factory does not know about these instances, so
 * FilterBank::clearBank()/remove() will not delete them -- they leak by
 * design until filter ownership is reworked (CLEANUP_PLAN P2-5).
 */
class DummyFilter : public toffy::Filter
{
   public:
    explicit DummyFilter(const std::string& filterName)
        : toffy::Filter("dummy", 0), wasCalled(false)
    {
        // Not strictly needed, but keeps the helper independent of the
        // uninitialised-member bug fixed separately in P0-5.
        bank(nullptr);
        name(filterName);
    }

    bool filter(const toffy::Frame& /*in*/, toffy::Frame& /*out*/) override
    {
        wasCalled = true;
        return true;
    }

    bool wasCalled;
};
