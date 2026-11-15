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

// The import filters' creators (P2-4).

#include <toffy/filter_init.hpp>

#include "toffy/import/dataimporter.hpp"
#include "toffy/import/importYaml.hpp"

namespace toffy {
namespace filters {

void initImportFilters(FilterFactory &factory)
{
    using namespace toffy::import;

    factory.registerCreator("dataImporter",
                            []() -> Filter * { return new DataImporter(); });
    factory.registerCreator("importYaml",
                            []() -> Filter * { return new ImportYaml(); });
}

}  // namespace filters
}  // namespace toffy
