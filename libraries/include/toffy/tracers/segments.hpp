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

#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "toffy/graphs/graph.hpp"
#include "toffy/skeletonizers/skeletonizerFactory.hpp"
//#include "rapidjson/document.h"

#include "toffy/tracers/segments.hpp"

/** a collection of skeleton segments (which are connected typically)
 */
class Segments : public std::vector<RasterSegment*>
{
   public:
    virtual void points(std::vector<RasterPoint*>& points) const
    {
        for (unsigned int i = 0; i < size(); i++) {
            (*this)[i]->points(points);
        }
    }

    void addUnique(RasterSegment* seg)
    {
        Segments& s = *this;
        for (size_t i = 0; i < s.size(); i++) {
            if ((s[i]->start == seg->start && s[i]->end == seg->end) ||
                (s[i]->start == seg->end && s[i]->end == seg->start))
                return;
        }
        push_back(seg);
    }

    virtual bool contains(const RasterPoint* start,
                          const RasterPoint* end) const;
};
