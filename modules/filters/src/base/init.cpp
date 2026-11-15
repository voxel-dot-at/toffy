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

// The base filters' creators (P2-4). Each of these was a branch in
// FilterFactory::createFilter()'s chain; the chain is gone and this is how they
// reach the factory now -- the same way a plug-in's filters do.
//
// The creators are captureless lambdas, which convert to `CreateFilterFn`
// (`Filter* (*)(void)`) exactly like the named `CreateX()` functions elsewhere
// do: the plug-in ABI is unchanged (X6), and the type name and the class it
// produces stay on one line instead of being split across a three-line function
// and a registration 40 lines below it.

#include <toffy/filter_init.hpp>

#include "toffy/base/amplitudeRange.hpp"
#include "toffy/base/backgroundsubs.hpp"
#include "toffy/base/cond.hpp"
#include "toffy/base/distAmpl.hpp"
#include "toffy/base/focus.hpp"
#include "toffy/base/nop.hpp"
#include "toffy/base/offset.hpp"
#include "toffy/base/ofsCorr.hpp"
#include "toffy/base/polar2cart.hpp"
#include "toffy/base/range.hpp"
#include "toffy/base/rectify.hpp"
#include "toffy/base/roi.hpp"

namespace toffy {
namespace filters {

void initBaseFilters(FilterFactory &factory)
{
    factory.registerCreator("offset", []() -> Filter * { return new OffSet(); });
    factory.registerCreator("distAmpl",
                            []() -> Filter * { return new DistAmpl(); });
    factory.registerCreator("offsetCorr",
                            []() -> Filter * { return new OffsetCorr(); });
    factory.registerCreator("polar2cart",
                            []() -> Filter * { return new Polar2Cart(); });
    factory.registerCreator("rectify",
                            []() -> Filter * { return new Rectify(); });
    factory.registerCreator("amplitudeRange",
                            []() -> Filter * { return new AmplitudeRange(); });
    factory.registerCreator("range", []() -> Filter * { return new Range(); });
    factory.registerCreator("nop", []() -> Filter * { return new Nop(); });
    factory.registerCreator("cond", []() -> Filter * { return new Cond(); });
    factory.registerCreator("focus", []() -> Filter * { return new Focus(); });
    factory.registerCreator("roi", []() -> Filter * { return new Roi(); });
    // Registered under the class's own id_name, not a literal copy of it: the
    // chain spelled 33 of its 37 types as string literals, which is how a type
    // name drifts away from the class that owns it.
    factory.registerCreator(BackgroundSubs::id_name,
                            []() -> Filter * { return new BackgroundSubs(); });
}

}  // namespace filters
}  // namespace toffy
