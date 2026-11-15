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

// The detection filters' creators (P2-4).

#include <toffy/filter_init.hpp>

#include "toffy/detection/blobs.hpp"
#include "toffy/detection/blobsDetector.hpp"
#include <toffy/detection/mask.hpp>
#include "toffy/detection/objectTrack.hpp"
#if defined(PCL_FOUND)
#include "toffy/detection/squareDetect.hpp"
#endif

namespace toffy {
namespace filters {

void initDetectionFilters(FilterFactory &factory)
{
    using namespace toffy::detection;

    factory.registerCreator("objectTrack",
                            []() -> Filter * { return new ObjectTrack(); });
    factory.registerCreator("blobs", []() -> Filter * { return new Blobs(); });
    factory.registerCreator(BlobsDetector::id_name,
                            []() -> Filter * { return new BlobsDetector(); });
    factory.registerCreator("mask", []() -> Filter * { return new Mask(); });

#if defined(PCL_FOUND)
    // Guarded because the chain guarded it, not because it needs PCL:
    // squareDetect.cpp is built unconditionally and contains no pcl at all, so
    // a PCL-less build compiles the filter and then refuses to name it. Kept as
    // it behaves so this commit moves types rather than adding them; filed with
    // exportYaml (same shape) as a follow-up.
    factory.registerCreator("squareDetect",
                            []() -> Filter * { return new SquareDetect(); });
#endif
}

}  // namespace filters
}  // namespace toffy
