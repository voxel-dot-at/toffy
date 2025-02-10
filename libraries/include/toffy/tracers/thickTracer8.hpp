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
#ifndef THICKTRACER8_HPP
#define THICKTRACER8_HPP
#include <vector>
//#include "rapidjson/document.h"
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include "toffy/tracers/segmentTracer.hpp"

class ThickTracer8 : public SegmentTracer
{
   public:
    ThickTracer8();
    virtual ~ThickTracer8();

    //virtual void configure(const rapidjson::Value& configObject);

    virtual void trace(const cv::Mat& mat);

    static SegmentTracer* Create();

    void setSkel(Skeletonizer* s) { reaper = s; }

   protected:
    void findEndpoints(const cv::Mat& in);

    /** fill a list with all segments that can be detected in mat, first
     * using find_endpoints to get a list of points, then using followLine
     * and thus changing the input image.
     *
     * The function returns both a list of detected points as well as the list of
     * segments.
     *
     * It resolves teh end points detected by followLine in the pts list and updates the
     * segment to point to the one found, or adds a new RP to the points list if not found.
     */
    virtual void followLines(cv::Mat& mat, std::vector<RasterPoint*>& pts,
                             std::vector<RasterSegment*>& segs);

    /** line tracing: from a starting point, find and follow neighbours to
     * either a crossing or an end point. Sets start and endpoint in the
     * raster segment passed.
     *
     * This function modifies the in matrix: it follows pixels set to '1' and changes
     * them to '3', marking them as visited. 0-areas are untouched.
     *
     * helper is a temporary storage for the end point;
     *
     */
    virtual void followLine(cv::Mat& mat, RasterPoint& start,
                            RasterSegment& seg);
    // the skeletonizer to use

    int neighbors;
    Skeletonizer* reaper;
};
#endif  // THICKTRACER_H
