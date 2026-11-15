/*
   Copyright 2018 Simon Vogl <svogl@voxel.at>
                  Angel Merino-Sastre <amerino@voxel.at>

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

// The 3d filters' creators (P2-4). This file is in `toffy_3d`'s source list,
// and that target exists only when PCL is found -- which is also the condition
// under which core calls `init3dFilters()`, so the registrations and the build
// cannot disagree the way the chain's `#if defined(PCL_FOUND)` blocks could.
//
// `exportcloud` is not here: `viewers/init.cpp` already registered it (and the
// chain registered it a second time -- the chain won, so deleting the chain
// branch changes nothing). One owner for the name, and it is the module that
// builds the class.

#include <toffy/filter_init.hpp>

#include "toffy/3d/groundprojection.hpp"
#include "toffy/3d/merge.hpp"
#include "toffy/3d/muxMerge.hpp"
#include "toffy/3d/sampleConsensus.hpp"
#include "toffy/3d/split.hpp"
#include "toffy/3d/transform.hpp"
#include "toffy/3d/xyz2pcl.hpp"

namespace toffy {
namespace filters {

void init3dFilters(FilterFactory &factory)
{
    using namespace toffy::filters::f3d;

    factory.registerCreator("transform",
                            []() -> Filter * { return new Transform(); });
    factory.registerCreator("merge", []() -> Filter * { return new Merge(); });
    factory.registerCreator("xyz2pcl",
                            []() -> Filter * { return new Xyz2Pcl(); });
    factory.registerCreator("split", []() -> Filter * { return new Split(); });
    factory.registerCreator("muxMerge",
                            []() -> Filter * { return new MuxMerge(); });
    factory.registerCreator("sampleConsensus",
                            []() -> Filter * { return new SampleConsensus(); });
    factory.registerCreator("groundprojection",
                            []() -> Filter * { return new GroundProjection(); });
}

}  // namespace filters
}  // namespace toffy
