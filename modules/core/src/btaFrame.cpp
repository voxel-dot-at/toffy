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

// Single definition of the BTA camera slot-name constants that btaFrame.hpp
// declares extern. See the comment there for why they are not defined in the
// header (P2-9): a namespace-scope `const` object has internal linkage, so
// defining them in an installed header gave every including translation unit
// its own object, its own dynamic initialiser and its own address.

#include "toffy/btaFrame.hpp"

namespace toffy {

const std::string btaMf = "mf";
const std::string btaIt = "it";
const std::string btaFc = "fc";

const std::string btaDepth = "depth";
const std::string btaAmpl = "ampl";

}  // namespace toffy
