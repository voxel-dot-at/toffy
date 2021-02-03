#ifndef THICKTRACER8_HPP
#define THICKTRACER8_HPP
#include <vector>
//#include "rapidjson/document.h"
#if OCV_VERSION_MAJOR >= 3
#include <opencv2/core.hpp>
#else
#include <opencv2/core/core.hpp>
#endif
#include "toffy/tracers/segmentTracer.hpp"

class ThickTracer8 : public SegmentTracer {
public:
    ThickTracer8();
    virtual ~ThickTracer8();

    //virtual void configure(const rapidjson::Value& configObject);

    virtual void trace(const cv::Mat& mat);

    static SegmentTracer * Create();

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
    virtual void followLines(cv::Mat& mat, std::vector<RasterPoint*>& pts, std::vector<RasterSegment*>& segs);

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
    virtual void followLine(cv::Mat& mat, RasterPoint& start, RasterSegment& seg);
    // the skeletonizer to use

    int neighbors;
    Skeletonizer* reaper;
};
#endif // THICKTRACER_H
