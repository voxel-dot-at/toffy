/*
 * SegmentTracer.h
 *
 *  Created on: Dec 13, 2012
 *      Author: simon
 */

#ifndef SEGMENTTRACER_H_
#define SEGMENTTRACER_H_

#include<vector>
#if OCV_VERSION_MAJOR >= 3
#include <opencv2/core.hpp>
#else
#include <opencv2/core/core.hpp>
#endif
#include "toffy/tracers/segments.hpp"
#include "toffy/graphs/graph.hpp"
#include "toffy/skeletonizers/skeletonizerFactory.hpp"
//#include "rapidjson/document.h"


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
