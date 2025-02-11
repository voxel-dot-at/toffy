/*
 * ThickTracer8.cpp
 *
 *  Created on: Fev 13, 2012
 *      Author: David
 */

#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

#include "toffy/tracers/thickTracer8.hpp"

using namespace cv;
using namespace std;
// using namespace rapidjson;

const bool dumpTracerDetails = false;
const bool trc = false;      // really low level stuff
const bool dumpFix = false;  // fixup stage

/****************************************UTILITY
 * FCT***************************************************/
static inline int countNeighbours(const cv::Mat& mat, int y, int x)
{
    // get line ptr
    const uchar* prev;
    const uchar* cur = mat.ptr<unsigned char>(y);
    const uchar* next;
    int count = 0;

    if (y > 0) {
        prev = mat.ptr<unsigned char>(y - 1);
        if (x > 0 && prev[x - 1]) count++;
        if (prev[x]) count++;
        if ((x < mat.cols - 1) && prev[x + 1]) count++;
    }

    if (x > 0 && cur[x - 1]) count++;
    if ((x < mat.cols - 1) && cur[x + 1]) count++;

    if (y < mat.rows - 1) {
        next = mat.ptr<unsigned char>(y + 1);
        if (x > 0 && next[x - 1]) count++;
        if (next[x]) count++;
        if ((x < mat.cols - 1) && next[x + 1]) count++;
    }

    return count;
}

// static int countDiagNeighbours(const cv::Mat& mat, int x, int y)
// {
//     // get line ptr
//     const uchar* prev = mat.ptr<unsigned char>(y - 1);
//     // const uchar* cur = mat.ptr<unsigned char>(y);
//     const uchar* next = mat.ptr<unsigned char>(y + 1);
//     int count = 0;

//     if (y) {
//         if (prev[x - 1]) count++;
//         if (prev[x + 1]) count++;
//     }
//     if (y < mat.rows - 1) {
//         if (next[x - 1]) count++;
//         if (next[x + 1]) count++;
//     }
//     return count;
// }

static bool hasUnvisitedNeighs(const cv::Mat& in, int x, int y, int neighCount)
{
    int a, b, c, d, e, f, g, h;
    const uchar* prv = in.ptr(y - 1);
    const uchar* cur = in.ptr(y);
    const uchar* nxt = in.ptr(y + 1);

    a = b = c = d = e = f = g = h = 0;

    if (y > 0) {
        if (x) h = prv[x - 1];

        a = prv[x];

        if (x < in.cols - 1) b = prv[x + 1];
    }
    if (x) g = cur[x - 1];
    if (x < in.cols - 1) c = cur[x + 1];

    if (y < in.rows - 1) {
        if (x) f = nxt[x - 1];
        e = nxt[x] == 1;
        if (x < in.cols - 1) d = nxt[x + 1];
    }

    if (y > 0) {
        if (x) h = prv[x - 1] == 1;

        a = prv[x] == 1;

        if (x < in.cols - 1) b = prv[x + 1] == 1;
    }
    if (x) g = cur[x - 1] == 1;
    if (x < in.cols - 1) c = cur[x + 1] == 1;

    if (y < in.rows - 1) {
        if (x) f = nxt[x - 1] == 1;
        e = nxt[x] == 1;
        if (x < in.cols - 1) d = nxt[x + 1] == 1;
    }

    return a + b + c + d + e + f + g + h;
}

/** internal helper: collect a list of neighbours - unset neighs are marked with
 * x==-1 returns the number of set pixels.
 *
 * this function looks only for unvisited pixels (value==1)!
 *
 * pixel arrangement (cf. Lam):
 * x4 x3  x2      prv
 * x5  X  x1      cur
 * x6 x7  x8      nxt
 */
static int getNeighs(const cv::Mat& in, int x, int y, int dir, Point neigh[8],
                     unsigned char* bits = 0)
{
    uchar x3, x2, x1, x8, x7, x6, x5, x4;
    const uchar* prv = in.ptr(y - 1);
    const uchar* cur = in.ptr(y);
    const uchar* nxt = in.ptr(y + 1);

    x3 = x2 = x1 = x8 = x7 = x6 = x5 = x4 = 0;

    if (y > 0) {
        if (x) x4 = prv[x - 1] == 1;

        x3 = prv[x] == 1;

        if (x < in.cols - 1) x2 = prv[x + 1] == 1;
    }
    if (x) x5 = cur[x - 1] == 1;
    if (x < in.cols - 1) x1 = cur[x + 1] == 1;

    if (y < in.rows - 1) {
        if (x) x6 = nxt[x - 1] == 1;
        x7 = nxt[x] == 1;
        if (x < in.cols - 1) x8 = nxt[x + 1] == 1;
    }

    if (x1) {
        neigh[0].x = x + 1;
        neigh[0].y = y;
    } else {
        neigh[0].x = -1;
    }

    if (x2) {
        neigh[1].x = x + 1;
        neigh[1].y = y - 1;
    } else {
        neigh[1].x = -1;
    }

    if (x3) {
        neigh[2].x = x;
        neigh[2].y = y - 1;
    } else {
        neigh[2].x = -1;
    }

    if (x4) {
        neigh[3].x = x - 1;
        neigh[3].y = y - 1;
    } else {
        neigh[3].x = -1;
    }

    if (x5) {
        neigh[4].x = x - 1;
        neigh[4].y = y;
    } else {
        neigh[4].x = -1;
    }

    if (x6) {
        neigh[5].x = x - 1;
        neigh[5].y = y + 1;
    } else {
        neigh[5].x = -1;
    }

    if (x7) {
        neigh[6].x = x;
        neigh[6].y = y + 1;
    } else {
        neigh[6].x = -1;
    }

    if (x8) {
        neigh[7].x = x + 1;
        neigh[7].y = y + 1;
    } else {
        neigh[7].x = -1;
    }

    if (bits != 0) {
        *bits = (x1 << 0) | (x2 << 1) | (x3 << 2) | (x4 << 3) | (x5 << 4) |
                (x6 << 5) | (x7 << 6) | (x8 << 7);
    }
    return x3 + x2 + x1 + x8 + x7 + x6 + x5 + x4;
}

/** get ALL neighbour points that are set (regardless if visited or not
 *
 */
static int getAllNeighs(const cv::Mat& in, int x, int y, Point neigh[8])
{
    int x3, x2, x1, x8, x7, x6, x5, x4;
    const uchar* prv = in.ptr(y - 1);
    const uchar* cur = in.ptr(y);
    const uchar* nxt = in.ptr(y + 1);
    // int idx = 0;

    x3 = x2 = x1 = x8 = x7 = x6 = x5 = x4 = 0;

    if (y > 0) {
        if (x) x4 = prv[x - 1];

        x3 = prv[x];

        if (x < in.cols - 1) x2 = prv[x + 1];
    }
    if (x) x5 = cur[x - 1];
    if (x < in.cols - 1) x1 = cur[x + 1];

    if (y < in.rows - 1) {
        if (x) x6 = nxt[x - 1];
        x7 = nxt[x];
        if (x < in.cols - 1) x8 = nxt[x + 1];
    }
    if (trc)
        cout << "ALLNEI " << x1 << x2 << x2 << x3 << x4 << x5 << x6 << x6 << x8
             << endl;
    if (x1) {
        neigh[0].x = x + 1;
        neigh[0].y = y;
    } else {
        neigh[0].x = -1;
    }

    if (x2) {
        neigh[1].x = x + 1;
        neigh[1].y = y - 1;
    } else {
        neigh[1].x = -1;
    }

    if (x3) {
        neigh[2].x = x;
        neigh[2].y = y - 1;
    } else {
        neigh[2].x = -1;
    }

    if (x4) {
        neigh[3].x = x - 1;
        neigh[3].y = y - 1;
    } else {
        neigh[3].x = -1;
    }

    if (x5) {
        neigh[4].x = x - 1;
        neigh[4].y = y;
    } else {
        neigh[4].x = -1;
    }

    if (x6) {
        neigh[5].x = x - 1;
        neigh[5].y = y + 1;
    } else {
        neigh[5].x = -1;
    }

    if (x7) {
        neigh[6].x = x;
        neigh[6].y = y + 1;
    } else {
        neigh[6].x = -1;
    }

    if (x8) {
        neigh[7].x = x + 1;
        neigh[7].y = y + 1;
    } else {
        neigh[7].x = -1;
    }

    return x3 + x2 + x1 + x8 + x7 + x6 + x5 + x4;
}

/**************************************ThickTracer8*********************************************/
ThickTracer8::ThickTracer8()
{
    SkeletonizerFactory* Factory = SkeletonizerFactory::Get();
    reaper = Factory->getSkeletonizer("k3mppskeletonizer");
    neighbors = 8;
    debugging = false;
}
/*
void ThickTracer8::configure(const Value &configObject)
{
    const char* skeletonizerClass = configObject["Skeletonizer"].GetString();
    neighbors = configObject["Neighbors"].GetInt();
    if(skeletonizerClass == 0 || (neighbors != 4 && neighbors != 8)){
        cout << "WHAT HAVE YOU DONE?\nYOU KILLED US ALL!" << endl;
        exit(0);
    }
    SkeletonizerFactory* Factory = SkeletonizerFactory::Get();
    if(reaper != 0){
        delete reaper;
        reaper = 0;
    }
    Debugging = configObject["Debug"].GetBool();
    reaper = Factory->getSkeletonizer(skeletonizerClass);
    reaper->configure(configObject);

    neighbors = 8;
}
*/

ThickTracer8::~ThickTracer8()
{
    delete reaper;
    reaper = 0;
}

void ThickTracer8::trace(const Mat& mat)
{
    SegmentTracer::trace(mat);

    if (!skeleton) {
        this->skeleton = new Mat();
    }
    // skeleton = 0;
    reaper->skeletonize(mat, *skeleton);

    if (debugging) {
        Mat sk;
        resize(*skeleton, sk, Size(0, 0), 2, 2, INTER_NEAREST);
        cout << "ThickTracer8 : Total : Found " << points.size() << " points"
             << endl;
        imshow("SK", sk > 0);
    }

    thickness = &reaper->thickness;

    findEndpoints(*skeleton);

    followLines(*skeleton, points, segments);

    if (debugging) {
        cout << "ThickTracer8 : Total : Found " << segments.size()
             << " Segments" << endl;
    }
    if (dumpTracerDetails) {
        cout << "ThickTracer8 - dump segs" << endl;
        for (size_t i = 0; i < segments.size(); i++) {
            cout << "seg " << i << "\t" << *segments[i] << endl;
        }
        for (size_t i = 0; i < points.size(); i++) {
            cout << "pt " << i << "\t" << *points[i] << endl;

            for (size_t j = 0; j < points[i]->segs.size(); j++) {
                cout << "\tseg " << j << "\t" << *points[i]->segs[j] << endl;
            }
        }
    }
}

SegmentTracer* ThickTracer8::Create() { return new ThickTracer8(); }

void ThickTracer8::findEndpoints(const cv::Mat& in)
{
    int ends = 0, crosses = 0;
    // Scan the Image
    const unsigned char* cur;
    cout << dec;
    // Rows
    for (int i = 0; i < in.rows; i++) {
        // get line ptr
        cur = in.ptr<uchar>(i);
        // Columns
        for (int j = 0; j < in.cols; j++) {
            // is a object pix?
            if (!cur[j]) {
                continue;
            }
            // is an Endpoint?
            int z = countNeighbours(in, i, j);

            // cout << "ct " << j <<","<<i << " " << z <<  endl;

            if (z == 1 || z == 0) {
                if (dumpTracerDetails) cout << "end " << j << "," << i << endl;
                addPoint(new RasterPoint(j, i, endPoint,
                                         reaper->thickness.at<uchar>(i, j)));
                ends++;
            } else if (z == 2) {
                if (false && dumpTracerDetails)
                    cout << "  inline " << j << "," << i << endl;
            } else if (z > 2) {
                if (trc) cout << "cross " << j << "," << i << endl;
                crosses++;
                addPoint(new RasterPoint(j, i, crossingPoint,
                                         reaper->thickness.at<uchar>(i, j)));
            }
        }
    }
    if (debugging) {
        cout << "ThickTracer8 : Found " << ends << " endpoints and " << crosses
             << " crossingpoints" << endl;
    }
}

#define X1 (1 << 0)
#define X2 (1 << 1)
#define X3 (1 << 2)
#define X4 (1 << 3)
#define X5 (1 << 4)
#define X6 (1 << 5)
#define X7 (1 << 6)
#define X8 (1 << 7)

void ThickTracer8::followLines(cv::Mat& mat, std::vector<RasterPoint*>& pts,
                               std::vector<RasterSegment*>& segs)
{
    Mat disMax;
    disMax.create(mat.rows, mat.cols, CV_8UC3);
    disMax.setTo(Scalar(128, 128, 128), *skeleton);

    int id = 0;
    int i;

    if (trc)
        for (i = 0; i < (int)pts.size(); i++) {
            RasterPoint& pt = *pts[i];
            if (pt.type != crossingPoint) {
                continue;
            }
            cout << " DUMP CROSSP " << pt << endl;
        }
    if (trc)
        for (i = 0; i < (int)pts.size(); i++) {
            RasterPoint& pt = *pts[i];
            if (pt.type == crossingPoint) {
                continue;
            }
            cout << " DUMP      P " << pt << endl;
        }

    // handle crossing points first:
    for (i = 0; i < (int)pts.size(); i++) {
        RasterPoint& pt = *pts[i];
        if (pt.type != crossingPoint) {
            continue;
        }

        if (trc) cout << " START CROSSP " << pt << endl;

        // crossing points are special: they can have multiple connections.
        //
        // collect all neigbouring crossing points first...:
        Point neigh[8];

        mat.at<char>(pt) = 3;

        getAllNeighs(mat, pt.x, pt.y, neigh);  // get all surrounding pixels

        for (size_t j = 0; j < 8;
             j++) {  // pick the first crossing point that we can get
            RasterPoint* helper = findPoint(neigh[j].x, neigh[j].y);
            if (!helper) continue;
            if (helper->type != crossingPoint) continue;
            if (trc) cout << "C-C chk " << *helper << endl;
            // now check if we have a segment already
            if (!segments.contains(&pt, helper)) {
                // nope, does not exist yet..:
                RasterSegment* newSeg = new RasterSegment();
                newSeg->id = id++;
                newSeg->start = &pt;
                newSeg->end = helper;
                newSeg->len = 1;
                mat.at<char>(neigh[j]) = 3;

                segs.push_back(newSeg);
                newSeg->start->segs.push_back(newSeg);
                newSeg->end->segs.push_back(newSeg);
                if (dumpTracerDetails)
                    cout << "C-C seg added " << *newSeg << endl;
            } else {
                if (dumpTracerDetails)
                    cout << "C-C seg already there to " << *helper << endl;
            }
        }

        int count = hasUnvisitedNeighs(mat, pt.x, pt.y, neighbors);
        bool hasNeighbours = count > 0;

        if (!hasNeighbours) continue;

        while (hasNeighbours) {
            if (trc)
                cout << " START FOLLOW " << pt << " NEIGHU " << hex << count
                     << dec << endl;

            RasterSegment* newSeg = new RasterSegment();
            RasterSegment& seg = *newSeg;
            seg.id = id++;
            followLine(mat, pt, seg);
            pt.visited = true;
            seg.end->visited = true;
            mat.at<char>(pt) = 3;

            segs.push_back(newSeg);
            seg.start->segs.push_back(newSeg);
            seg.end->segs.push_back(newSeg);

            // pt.segs.push_back(newSeg);

            count = hasUnvisitedNeighs(mat, pt.x, pt.y, neighbors);
            hasNeighbours = count > 0;
        }
    }
    if (trc) cout << "ENDPTS-------" << endl;
    // then go for end points; this triggers end--end lines w/o crossingpoints
    for (i = 0; i < (int)pts.size(); i++) {
        RasterPoint& pt = *pts[i];
        if (pt.type != endPoint || pt.visited) {
            continue;
        }

        int count = hasUnvisitedNeighs(mat, pt.x, pt.y, neighbors);
        bool hasNeighbours = count > 0;

        if (!hasNeighbours) continue;

        while (hasNeighbours) {
            RasterSegment* newSeg = new RasterSegment();
            RasterSegment& seg = *newSeg;
            seg.id = id++;
            followLine(mat, pt, seg);
            pt.visited = true;
            mat.at<char>(pt) = 3;

            segs.push_back(newSeg);
            seg.start->segs.push_back(newSeg);
            seg.end->segs.push_back(newSeg);

            // pt.segs.push_back(newSeg);

            count = hasUnvisitedNeighs(mat, pt.x, pt.y, neighbors);
            hasNeighbours = count > 0;
        }
    }
}

void ThickTracer8::followLine(cv::Mat& in, RasterPoint& start,
                              RasterSegment& seg)
{
    RasterPoint* helper;
    int len = 1;      // start always counts.
    int partLen = 0;  // length between stuetzpunkten
    int x, y;

    x = start.x;
    y = start.y;

    if (dumpTracerDetails)
        cout << "trc start " << x << "," << y << "------" << endl;

    seg.start = &start;
    // chain code : 0 is up:
    // 7 0 1
    // 6 X 2
    // 5 4 3

    int dir = 0;
    Point neigh[8];
    unsigned char bits;
    int count;
    Point last = start;

    // Find first point
    count = getNeighs(in, x, y, dir, neigh, &bits);
    // Be sure to enter the loop!
    if (count > 1) count = 1;
    while (count == 1) {
        // mark point as visited.
        in.at<char>(y, x) = 3;

        // Find the next point
        int i;
        for (i = 0; i < 8; i++) {
            if (neigh[i].x >= 0) {
                break;
            }
        }

        last = Point(x, y);

        // Move to the next point
        Point p = neigh[i];
        x = p.x;
        y = p.y;

        if (false && dumpTracerDetails)
            cout << "  trc nxt: " << p << " " << count << " b=" << hex
                 << (int)bits << dec << " i=" << i << endl;

        helper = findPoint(x, y);
        if (helper && helper->type == crossingPoint) {
            // unconditial stop - crossingPoint has been visited in other iterations.
            if (trc) cout << "  trc: stop at X point " << *helper << endl;
            break;
        }

        // Increase Segment lenght
        len++;
        partLen++;
        if (keepAll) {
            seg.pts.push_back(
                new RasterPoint(x, y, inLine, thickness->at<char>(y, x)));
        } else {
            // Add points
            if (partLen >= 30) {
                partLen = 0;
                seg.pts.push_back(new RasterPoint(x, y, cornerPoint,
                                                  thickness->at<char>(y, x)));
            }
        }
        if (false && dumpTracerDetails)
            cout << "  trc: " << p << " " << count << " b=" << hex << (int)bits
                 << dec << endl;
        if (false) {  // single-step tracing
            Mat ii;
            resize(in, ii, Size(0, 0), 4, 4, INTER_NEAREST);
            imshow("IN", ii * 50);
        }

        // Find next Point
        count =
            getNeighs(in, p.x, p.y, dir, neigh, &bits);  // Check for next pixel

        if (dumpTracerDetails)
            cout << "  trc nxt2: " << p << " " << count << " b=" << hex
                 << (int)bits << dec << " count=" << count << endl;
    }
    // end of segment...
    if (dumpTracerDetails)
        cout << "trc fin @" << x << "," << y << " count " << count << endl;

    // we hit a stage where we have any number of neighbours (!=1).
    // as we count only unvisited points, this could mean that we
    // a) really have 0 pts -> endPoint ; wil be found with findPoint()
    // b) have count=0 but a visited crossing points in near -> connect
    // c) have >1 unvisited neighbour (2+); in this case,
    //       it makes sense to search for an existing crossing point.

    // Find the RasterPoint instantiated at this end of the segment
    helper = findPoint(x, y);
    // Shoudn't be happening!
    if (helper) {
        if (helper->type == endPoint) {
            if (trc)
                cout << " helper is endpoint "
                     << endl;  // helper->visited = true;
        } else if (trc)
            cout << " helper is not endpoint " << helper->type << endl;
    } else {
        cout << "WRN no endpoint found for " << x << "," << y << " "
             << seg.pts.size() << endl;
        // -> there must be a crossing point nearby
        vector<RasterPoint*> neighs;

        count = getAllNeighs(in, x, y, neigh);  // get all surrounding pixels
        // bool found = false;
        for (size_t i = 0; i < 8;
             i++) {  // pick the first crossing point that we can get
            if (neigh[i].x < 0) continue;
            helper = findPoint(neigh[i].x, neigh[i].y);
            if (trc && helper) cout << "  TST " << *helper << endl;
            if (helper && helper->type == crossingPoint) {
                // now we have a candidate; it COULD be in our chain already
                if (last != *helper) {
                    // found = true;
                    break;
                } else {
                    if (trc) cout << "   TST CHAINED! " << *helper << endl;
                }
            }
        }

        if (!helper) {
            // we're at an end point, p.ex. at the image border. create one.
            helper = new RasterPoint(x, y, endPoint, thickness->at<char>(y, x));
            cout << "no help - add end at " << *helper << endl;
            addPoint(helper);
        }
    }

    in.at<char>(y, x) = 3;
    // helper->visited = true;
    seg.end = helper;
    seg.len = len;
}

// static bool ptContainsSegTo(RasterPoint* source, RasterPoint* neigh)
// {
//     for (size_t i = 0; i < source->segs.size(); i++) {
//         RasterSegment* seg = source->segs[i];
//         if (seg->start == neigh || seg->end == neigh) return true;
//     }
//     return false;
// }
