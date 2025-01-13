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


#ifndef SEGMENTTRACER_H_
#define SEGMENTTRACER_H_

#include<vector>

#include <opencv2/core.hpp>

#include "toffy/tracers/segments.hpp"
#include "toffy/graphs/graph.hpp"
#include "toffy/skeletonizers/skeletonizerFactory.hpp"


/** Abstract class for all pixel to line tracers. First skeletonizes the image.
 *
 */
class SegmentTracer {
public:
    SegmentTracer();
    virtual ~SegmentTracer();

    std::vector<RasterPoint*> points;
    Segments segments;
    cv::Mat* skeleton;
    cv::Mat* thickness;

    //virtual void configure(const rapidjson::Value& configObject) = 0;

    virtual void reset();

    /** the super class does nothing but initialize internal data structures.
     * need to be called first from derived classes.
     */
    virtual void trace(const cv::Mat& skeleton);

    virtual const Segments& getSegments() const { return segments; }
    virtual Segments& getSegments() { return segments; }

    RasterPoint* findPoint(int x, int y);

    /** find all points up to a maximum (manhattan) distance, including x,y itself
     */
    void findPoints(int x, int y, std::vector<RasterPoint*>& hits, int maxDistance);

    /** find all points at a given (manhattan) distance
     */
    void findPointsAt(int x, int y, std::vector<RasterPoint*>& hits, int distance);

    /** find all points that are on diagonals and distance 2 (manhattan)
     */
    void findPointsDiag(int x, int y, std::vector<RasterPoint*>& hits);

    /** should all intermediate points on a segment be kept or only key points?
     * @param keep - keeps all points if true
     */
    void setKeepAllPoints(bool keep) { keepAll = keep; }

    void debug(bool dbg) { debugging = dbg; }
    
    
 protected:
    std::vector<std::vector<RasterPoint*> > pointList; // ordered by x coordinate

    /**
     * register a new point for lookup; adds to points and pointList
     */
    void addPoint(RasterPoint* rp);
    bool debugging;
    bool keepAll;
};

#endif /* SEGMENTTRACER_H_ */
