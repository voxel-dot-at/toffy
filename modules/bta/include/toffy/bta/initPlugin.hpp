/*
   Copyright 2012-2021 Simon Vogl <svogl@voxel.at> VoXel Interaction Design - www.voxel.at

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

#include <toffy/common/plugins.hpp>
#include "toffy/bta/bta.hpp"

#ifdef WITH_CONTROL
    #include <toffy_web/controllerFactory.hpp>
#endif

namespace toffy {
  namespace bta {
    void init(toffy::FilterFactory *ff);

#   ifdef WITH_CONTROL
    void initUI(control::ControllerFactory *cf);
#   endif
  }
}
