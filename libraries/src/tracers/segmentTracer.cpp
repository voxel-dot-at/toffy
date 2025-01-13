/*
 * SegmentTracer.cpp
 *
 *  Created on: Dec 13, 2012
 *      Author: simon
 */

#include <opencv2/imgproc/types_c.h>
#if OCV_VERSION_MAJOR >= 3
#include <opencv2/highgui.hpp>
#else
#include <opencv2/highgui/highgui.hpp>
#endif

#include "toffy/tracers/segmentTracer.hpp"

using namespace cv;
using namespace std;
//using namespace rapidjson;

/****************************************SEG TRACER************************************************************/
SegmentTracer::SegmentTracer() : skeleton(0), thickness(0), keepAll(false)
{
    debugging = false;
}

SegmentTracer::~SegmentTracer()
{
    reset();
    delete skeleton;
}

void SegmentTracer::reset()
{
    for (unsigned int i = 0; i < segments.size(); i++) {
        delete segments[i];
        segments[i] = 0;
    }
    segments.clear();
    for (unsigned int i = 0; i < points.size(); i++) {
        delete points[i];
        points[i] = 0;
    }
    points.clear();
    pointList.clear();
}

void SegmentTracer::trace(const Mat& skel) { pointList.resize(skel.cols); }

void SegmentTracer::addPoint(RasterPoint* rp)
{
    // register new point:
    if (debugging) {
        cout << "SegTracer : add " << *rp << endl;
    }
    points.push_back(rp);
    pointList[rp->x].push_back(rp);
}

void SegmentTracer::findPoints(int x, int y, std::vector<RasterPoint*>& hits,
                               int maxDistance)
{
    for (int i = x - maxDistance; i <= x + maxDistance; i++) {
        if (i < 0 || (unsigned int)i >= pointList.size()) continue;
        vector<RasterPoint*>& pts = pointList[i];
        for (unsigned int j = 0; j < pts.size(); j++) {
            RasterPoint* rp = pts[j];
            if (rp->distance(x, y) <= maxDistance) {
                hits.push_back(rp);
            }
        }
    }
}

void SegmentTracer::findPointsAt(int x, int y, std::vector<RasterPoint*>& hits,
                                 int distance)
{
    for (int i = x - distance; i <= x + distance; i++) {
        if (i < 0 || (unsigned int)i >= pointList.size()) continue;
        vector<RasterPoint*>& pts = pointList[i];
        for (unsigned int j = 0; j < pts.size(); j++) {
            RasterPoint* rp = pts[j];
            if (rp->distance(x, y) == distance) {
                hits.push_back(rp);
            }
        }
    }
}

void SegmentTracer::findPointsDiag(int x, int y,
                                   std::vector<RasterPoint*>& hits)
{
    for (int i = x - 1; i <= x + 1; i++) {
        if (i < 0 || (unsigned int)i >= pointList.size()) continue;
        vector<RasterPoint*>& pts = pointList[i];
        for (unsigned int j = 0; j < pts.size(); j++) {
            RasterPoint* rp = pts[j];
            if (rp->y == y - 1 || rp->y == y + 1) {
                hits.push_back(rp);
            }
        }
    }
}

RasterPoint* SegmentTracer::findPoint(int x, int y)
{
    if (x < 0 || (unsigned int)x > pointList.size()) return 0;

    vector<RasterPoint*>& pts = pointList[x];
    for (unsigned int i = 0; i < pts.size(); i++) {
        RasterPoint* rp = pts[i];
        if (rp->x == x && rp->y == y) {
            return rp;
        }
    }
    return 0;
}
