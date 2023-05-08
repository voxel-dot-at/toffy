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

#include "toffy/graphs/raster.hpp"
#include "toffy/graphs/boundingBox.hpp"

#include <vector>

class TouchInfo {
public:
    TouchInfo(Karterl* k=0, cv::Point* p=0) : karterl(k),point(p),numPoints(0),isArrow(false) {}

    TouchInfo(const TouchInfo& ti) : karterl(ti.karterl),point(ti.point),numPoints(ti.numPoints),isArrow(ti.isArrow) {}

    TouchInfo& operator=(const TouchInfo& ti) {
        karterl = ti.karterl;
        point = ti.point;
        return *this;
    }

    Karterl* karterl;
    cv::Point* point;
    
    int numPoints;
    int isArrow;
};



enum segmentType {
	ign,					// eat me.
	belongsToMarker,		// marker stuff
	startsInRoi,		// something that could be the start of a connector
	potentialConnector,
	isBlob,					// roundish blobs, fill >10%
	isLine,					// line segments
	borderJunk 				// blobs on the image border
};

/** a graph is a collection of one or more segments (individual skeletons) that are not touching but belong together.
 */
class Graph {
public:
	Graph():id(0),touchesRoi(false),leavesRoi(false),isLoop(false),isConnector(false) {
		id=count++;
	};

	Graph(const Graph&g): id(g.id),segs(g.segs),karterlRois(g.karterlRois),
			      touchesRoi(g.touchesRoi),leavesRoi(g.leavesRoi),
			      isLoop(g.isLoop),isConnector(g.isConnector) {
	};

	Graph(std::vector<RasterSegment*>& segs): segs(segs),
						  touchesRoi(false),leavesRoi(false),
						  isLoop(false),isConnector(false) {
	    id = count++;
	};

	virtual ~Graph(){}


	Graph& operator=(const Graph& g) {
	    id = g.id;
	    segs = g.segs;
	    karterlRois = g.karterlRois;
	    touchesRoi = g.touchesRoi;
	    leavesRoi = g.leavesRoi;
	    isLoop = g.isLoop;
	    isConnector = g.isConnector;
	    return *this;
	}

	int id;
	std::vector<RasterSegment*> segs;
	std::map<std::string, TouchInfo> karterlRois;

	segmentType t;
	bool touchesRoi; // touches a karterl roi == map not empty
	bool leavesRoi; // leaves a roi -->
	bool isLoop; // leaves a roi --> enters same roi again
	bool isConnector; // leaves a roi --> enters other roi
	bool isCon; //for testing purpose

	virtual inline void add(RasterSegment* me) { segs.push_back(me); }

	BoundingBox getBoundingBox();

	virtual bool isNear(RasterSegment* checkMe, int fuzz=2, int minFuzz=5) const {
		//std::cout << "ck " << segs.size() << " ";
		for (unsigned int i=0;i<segs.size();i++) {
			if ( checkMe->isNear(segs[i],fuzz,minFuzz))
				return true;
		}
		return false;
	}

	virtual RasterSegment* nearest(RasterSegment* checkMe) const 
	{
		//std::cout << "ck " << segs.size() << " ";
		int min=5000;
		RasterSegment* near = 0;
		for (unsigned int i=0;i<segs.size();i++) {
			int dist = checkMe->minDistance(segs[i]);
			if (dist<min) {
				min = dist;
				near = segs[i];
			}
		}
		return near;
	}

	virtual cv::Rect bb() 
	{
		cv::Rect box;
		if (segs.size()==0) {
			std::cerr<<"ERR graph.bb size 0"<<std::endl;
			box.width=-1;
			box.height=-1;
			return box;
		}
		box = segs[0]->bb();

		for (unsigned int i=1;i<segs.size();i++) {
			segs[i]->mergeBb(box);
		}
		return box;
	}

	virtual void points(std::vector<cv::Point*>& points) const
	{
		for (unsigned int i=0;i<segs.size();i++) {
			segs[i]->points(points);
		}
	}

	virtual void points(std::vector<RasterPoint*>& points, bool unVisitedOnly=false, bool addInlinePoints=false) const
	{
		for (unsigned int i=0;i<segs.size();i++) {
		    segs[i]->points(points, unVisitedOnly, addInlinePoints);
		}
	}

	virtual void unPrunedPoints(std::vector<RasterPoint*>& points, bool unVisitedOnly=false) const
	{
		for (unsigned int i=0;i<segs.size();i++) {
			if (!segs[i]->pruned)
			    segs[i]->points(points, unVisitedOnly);
		}
	}

	/** compute center of gravity
	 *
	 */
	virtual cv::Point center()
	{
		cv::Point p;
		std::vector<cv::Point*> pts;
		points(pts);
		p.x = pts[0]->x;
		p.y = pts[0]->y;
		for (unsigned int i=1;i<pts.size();i++) {
			p.x += pts[i]->x;
			p.y += pts[i]->y;
		}
		p.x /= pts.size();
		p.y /= pts.size();
		return p;
	}

	/** compute approx. length
	 *
	 */
	virtual int length() const 
	{
		int len=0;
		for (unsigned int i=0;i<segs.size();i++) {
			len += segs[i]->len;
		}
		return len;
	}

	// assert: segments are ordered (l->r)
	virtual RasterPoint* rightMost() 
	{
		int x=-1;
		RasterPoint*rr=0;
		for (unsigned int i=0;i<segs.size();i++) {
			RasterSegment& seg = *segs[i];
			if (seg.end->x >= x) {
				x = seg.end->x;
				rr = seg.end;
			}
		}
		return rr;
	}
    
	virtual int meanThickness() const 
	{
		int mean=0;
		for (unsigned int i=0;i<segs.size();i++) {
			mean+=segs[i]->start->thick;
			mean+=segs[i]->end->thick;
		}
		return mean / (2*segs.size());
	}
    
	void erase(const RasterSegment* seg)
	{
		std::vector<RasterSegment*>::iterator iter = segs.begin();
		while (iter != segs.end()) {
			if ( *iter == seg) {
				segs.erase(iter);
				return;
			}
			iter++;
		}
	}
private:
	static int count;
};
