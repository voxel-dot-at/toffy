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

// The smoothing filters' creators (P2-4).
//
// `kalmanAverage` is deliberately NOT registered here, and the reason is not
// that it cannot be: this directory is compiled with `-DOPENCV_TRACKING=1` when
// OpenCV's tracking module is found (see this directory's CMakeLists.txt), so a
// guard here would actually work. It did not work where it lived -- the chain
// in modules/core tested `OPENCV_TRACKING`, a macro core never sees, so
// "kalmanAverage" was a type name no configuration could use while the class
// sat compiled and linked inside libtoffy.so.
//
// Registering it here changes which configs load, so it is not part of a commit
// whose whole claim is that the resolvable set did not move. It is its own
// change, with `reprojectopencv` and `reprojectpcl` (same bug class, different
// macro); `tests/test_filter_registry.cpp` pins all three as unreachable until
// then.

#include <toffy/filter_init.hpp>

#include "toffy/smoothing/average.hpp"
#include "toffy/smoothing/bilateral.hpp"

namespace toffy {
namespace filters {

void initSmoothingFilters(FilterFactory &factory)
{
    using namespace toffy::filters::smoothing;

    factory.registerCreator("bilateral",
                            []() -> Filter * { return new Bilateral(); });
    factory.registerCreator("average",
                            []() -> Filter * { return new Average(); });
}

}  // namespace filters
}  // namespace toffy
