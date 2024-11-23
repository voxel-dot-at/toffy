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

#include <toffy/toffy_export.h>

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <streambuf>

#if OCV_VERSION_MAJOR >= 3
#include <opencv2/core.hpp>
#else
#include <opencv2/core/core.hpp>
#endif

class Marker;
class Karterl;
class RasterSegment;
class Graph;

enum rasterPointType { inLine, endPoint, crossingPoint, cornerPoint};

class TOFFY_EXPORT RasterPoint: public cv::Point_<int> {
public:
    RasterPoint(): cv::Point_<int>(-1,-1),id(-1),type(inLine),thick(0),
    	count(0),visited(false),segs(0) {
    	id = counter++;
    }

    RasterPoint(int x,int y, rasterPointType t=inLine,int thickness=0) :
        cv::Point_<int>(x,y),id(-1),type(t),thick(thickness),count(0),
        visited(false),segs(0) {
    	id = counter++;
    }

    RasterPoint(const cv::Point2i& p): cv::Point_<int>(p.x,p.y),id(-1),type(inLine),thick(0),
				       count(0),visited(false),segs(0) {}

    virtual ~RasterPoint() {}

    RasterPoint(const RasterPoint& o) :
    	cv::Point_<int>(o),id(o.id),type(o.type),thick(o.thick),count(o.count),
    	visited(o.visited),segs(o.segs) {
    	std::cout << "cp " << x << "," << y << " " << std::flush;
    }

    operator cv::Point2i() const { return cv::Point2i(x,y); }
    operator cv::Point2f() const { return cv::Point2f(x,y); }
    operator cv::Vec2f() const { return cv::Vec2f(x,y); }
    
    inline int distance(const cv::Point* b) const {
        return abs(x - b->x) + abs(y - b->y);
    }

    inline int distance(int bx, int by) const {
        return abs(x - bx) + abs(y - by);
    }

    virtual operator std::string() const {
        std::stringstream sb;
        sb << "(" << id << ":" << x << "," << y << ")";
        return sb.str();
    }

    int id;
    rasterPointType type;          ///< type (referring to segments/graph - endpoint, crossingpint,... )
    int thick;                     ///< thickness at p
    int count;                     ///<
    bool visited;                  ///< for graph traversal algorithms
    int flags;                     ///< for custom use like classification,...
    std::vector<RasterSegment*> segs;

    virtual bool operator==(const RasterPoint& lhs){
        if(lhs.x == this->x && lhs.y == this->y){
            return true;
        }
        return false;
    }

private:
    static int counter;
};

inline int max(int a,int b) {
    return ( (a)>(b)?(a):(b));
}


class RasterSegment {
    static int RID;
public:
    RasterSegment(RasterPoint& start=unknown, RasterPoint& end=unknown);

    RasterSegment(const RasterSegment& o);

    virtual ~RasterSegment();

    int id; // segment number, just in case
    RasterPoint* start; 
    RasterPoint* end;

    std::vector<RasterPoint*> pts;
    
    int len; // number of pixels
    //struct _rSeg* next[8]; // segments attached to s[1]
	
    Graph* myGraph;
    bool pruned;


    /** return the other end point of this segment
     */
    const RasterPoint* otherEnd(const RasterPoint* oneEnd) const {
	if (oneEnd==start)
	    return end;
	else
	    return start;
    }

    RasterPoint* otherEnd(RasterPoint* oneEnd) const {
	if (oneEnd==start)
	    return end;
	else
	    return start;
    }

    /** find the next point of interest; this is either the other end, or - if interim points exist, the next one in range.
     */
    const RasterPoint* nextInteresting(const RasterPoint* oneEnd, int dist=1) const {
	if (oneEnd==start) {
	    if ((int)pts.size() > dist )
		return pts[dist];
	    else
		return end;
	} else {
	    if ((int)pts.size() > dist )
		return pts[ pts.size()-1-dist ];
	    else
		return start;
	}
    }

    inline int manhattan() const {
        return abs(start->x - end->x)+abs(start->y - end->y);
    }

    inline int manhattan(cv::Point* a, cv::Point* b) const {
        return abs(a->x - b->x) + abs(a->y - b->y);
    }

    bool isNear(RasterSegment* checkMe, int fuzz=2,int minFuzz=5) const {
        if ( manhattan(start,checkMe->start)
            < minFuzz+fuzz*max(start->thick,checkMe->start->thick)

            || manhattan(start,checkMe->end)
            < minFuzz+fuzz*max(start->thick,checkMe->end->thick)

            || manhattan(end,checkMe->start)
            < minFuzz+fuzz*max(end->thick,checkMe->start->thick)

            || manhattan(end,checkMe->end)
            < minFuzz+fuzz*max(end->thick,checkMe->end->thick)
            )
            return true;
        return false;
    }

    int minDistance(RasterSegment* checkMe) const {
        int ss= manhattan(start,checkMe->start);
        int se= manhattan(start,checkMe->end );
        int es= manhattan(end ,checkMe->start);
        int ee= manhattan(end ,checkMe->end );
		int min1 =ss<se ? ss:se;        
		int min2 =es<ee ? es:ee;
		if (min1<min2)
			return min1;
		else 
			return min2;        
	}

	void nearest(RasterSegment* checkMe, RasterPoint*& a, RasterPoint*& b, int& dStart, int&dEnd) {
        int ss= manhattan(start,checkMe->start);
        int se= manhattan(start,checkMe->end );
        int es= manhattan(end ,checkMe->start);
        int ee= manhattan(end ,checkMe->end );
		int min1 =ss<se ? ss:se;        
		int min2 =es<ee ? es:ee; 

		if (min1 < min2) {
			a = start;
            dStart = ss;
            dEnd = se;
			if (ss < se) {
				b=checkMe->start;
			} else {
				b=checkMe->end;
			}
		} else {
			a = end;
            dStart = es;
            dEnd = ee;
			if (es < ee) {
				b=checkMe->start;
			} else {
				b=checkMe->end;
			}
		}
	}
	
	RasterPoint* nearest(RasterSegment* checkMe) {
        int ss= manhattan(start,checkMe->start);
        int se= manhattan(start,checkMe->end );
        int es= manhattan(end ,checkMe->start);
        int ee= manhattan(end ,checkMe->end );
		int min1 =ss<se ? ss:se;        
		int min2 =es<ee ? es:ee; 

		if (min1<min2) 
			return start;
		else 
			return end;
	}
	
	RasterPoint* nearest(cv::Point* p) {
        int ss= manhattan(start,p);
        int ee= manhattan(end , p);

		if (ss < ee) 
			return start;
		else 
			return end;
	}
	
    virtual void points(std::vector<cv::Point*>& points) const;
    virtual void points(std::vector<RasterPoint*>& points) const;
    virtual void points(std::vector<RasterPoint*>& points, bool unVisitedOnly=false, bool addInlinePoints=false) ;

    virtual void points(std::vector<cv::Point>& points) const;

    cv::Rect bb() const {
        cv::Rect r;
        r.width = abs(start->x-end->x);
        r.height =  abs(start->y - end->y);
        r.x = start->x < end->x ? start->x : end->x;
        r.y = start->y < end->y ? start->y : end->y;

        return r;
    }

    void mergeBb(cv::Rect& r) const {
        cv::Rect self = bb();
        //cv::Point tl = self.br();
        cv::Point self_br = self.br();
        cv::Point r_br = r.br();

        if (r.x>= self.x) {
            r.x = self.x;
        }
        if (r.y>=self.y) {
            r.y = self.y;
        }

        if (r_br.x <= self_br.x) {
            r_br.x = self_br.x;
        }
        if (r_br.y <= self_br.y) {
            r_br.y = self_br.y;
        }
        r.width = r_br.x - r.x;
        r.height = r_br.y - r.y;
    }
    /** a segment is a leaf, if only one of its endpoints 
     * is connected to the graph
     */
    bool isLeaf() const {
        return start->segs.size()==1 || end->segs.size()==1;
    }
    
    operator std::string() const;

    void extend();
private:
    static RasterPoint unknown;
};

/**
 * utility functions
 */
extern void paintSegment(cv::Mat& disMax, const RasterSegment& seg,
							cv::Scalar color = cv::Scalar(0, 0, 255), int thickness = 1);


std::ostream& operator<<(std::ostream& o, const RasterPoint& pt );
std::ostream& operator<<(std::ostream& o, const RasterSegment& seg );
