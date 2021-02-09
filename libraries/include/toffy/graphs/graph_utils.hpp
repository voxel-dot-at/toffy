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
/**
 * set of utility functions - used by handDetect
 */
#pragma once
#include <vector>

#include <toffy/graphs/graph.hpp>


/** find farthest point; ignores segment structure, uses geo information only.
 */
static RasterPoint* findFarthestPoint(const cv::Point2i& ref, const std::vector< RasterPoint*>& points )
{
    /*RasterPoint r;
    r.x=ref.x;
    r.y=ref.y;
    RasterPoint* tmp = &r;
    */
    RasterPoint* tmp = points[0];
    
    for  (size_t i=0 ; i<points.size() ; i++) {
	if (points[i]->type != endPoint)
	    continue;
	tmp = points[i];
	break;
    }

    for  (size_t i=0 ; i<points.size() ; i++) {
	RasterPoint& rp = *points[i];
	
	if (rp.type != endPoint)
	    continue;
	if (norm(ref - rp) >= norm(ref - *tmp)) {
	    tmp = points[i];
	} 
    }
    return tmp;
}

template<typename T> T* findNearestPoint(cv::Point2i& ref, 
				     const std::vector<T*>& points, 
					   bool sorted=true,
					   bool onlyEndpoints=true)
{
    T* min = points[0];
    float dist=1000.;
    float last=1000.;

    for  (size_t i=0 ; i<points.size() ; i++) {
	T& rp = *points[i];
	
	if (onlyEndpoints && rp.type != endPoint )
	    continue;

	float d = norm(ref-rp);
	if (d<dist) {
	    min = points[i];
	    dist = d;

	    if (sorted && d>last) {
		// distances are increasing - stop here.
		return min;
	    }
	}
	last = d;
    }
    return min;
}


/**
go from tail(arm trunk) to tip(finger)...

vector go(seg, tail, tip) {
t=tail
s=currrent segment

n=s.nextEnd()

if n == tip
   return seg;

for i in nextEnd.segments
   if i == s  // current seg
     continue
     
   v = go(s, tip)
   if (v!=nil)
       return s | v
next
return vector() // empty set - no go
}

*/

/** get all the points in list in geometrically correct order.
 * if maxPoints is >0, this will stop as soon as the result list is over the number of points.
 * NB that we're adding segment by segment, so the actual number points returned may be well bigger
 */
static std::vector<const RasterPoint* >  getPtsSorted (const RasterPoint* from, 
						  const RasterPoint* /*to*/,
						  const std::vector<const RasterSegment* >& list,
						  int maxPoints=-1)
{
    std::vector<const RasterPoint* > points;
    const RasterPoint* current = from;
    int count=0;

    for (size_t i=0;i<list.size();i++) {
	const RasterSegment* seg = list[i]; 
	const RasterPoint* next = seg->otherEnd(current);
	std::vector<RasterPoint* > pts;

	list[i]->points(pts);

	if ( seg->start == current ) { // points are in order
	    for (size_t j=0 ; j<pts.size() ; j++) {
		points.push_back(pts[j]);
	    }
	} else {
	    for (int j=(int)pts.size()-1 ; j >= 0 ; j--) {
		points.push_back(pts[j]);
	    }
	}
	count += pts.size();
	if (maxPoints>0 && count > maxPoints)
	    break;
	current = next;
    }


    return points;
}

static int rec=0;
static bool dbgTrace=false;

static void plotCon (cv::Mat& img, cv::Scalar color, const RasterPoint* from)
{
    if (from->visited) {
	return;
    }
    ((RasterPoint*)from)->visited=true;

    for (size_t i=0;i<from->segs.size();i++) {
	const RasterPoint* next = from->segs[i]->otherEnd(from);

	line(img, *from, *next, color);

	plotCon(img, color, next);
    }
}

static void plotSegs (cv::Mat& img, const std::vector<RasterSegment*>& segs, cv::Scalar color)
{
    for (size_t i=0;i<segs.size();i++) {
	line(img, *segs[i]->start, *segs[i]->end, color);
    }
}

// helper function: find connection from a to b (depth first search)
static std::vector<const RasterSegment* >  getCon (const RasterPoint* from, const RasterPoint* to)
{
    using namespace std;
    std::vector<const RasterSegment* > chain;
    if (dbgTrace) {
	for (int i=0;i<rec;i++) cout << "\t";
	cout << "C f " << *from << " s=" << from->segs.size() << " v=" << from->visited;
	if (from == to) cout << "***";
	cout << endl;
    }
    if (from->visited) {
	if (dbgTrace) {
	    for (int i=0;i<rec;i++) cout << "\t";
	    cout << "FOUND vis! " << *from->segs[0]->otherEnd(from) <<  endl;
	}
	return chain;
    }
    ((RasterPoint*)from)->visited=true;

    for (size_t i=0;i<from->segs.size();i++) {
	const RasterPoint* next = from->segs[i]->otherEnd(from);

	if (dbgTrace) {
	    for (int k=0;k<rec;k++) cout << "\t";
	    cout << "  ...> " << *next << " p " << from->segs[i]->pruned << "\t" 
		 << "\t" << *from << endl;
	}

	//final segment:
	if (next==to ||
	    ( to && 
	      ( to->type==inLine &&
		find(from->segs[i]->pts.begin(), 
		     from->segs[i]->pts.end(), to) 
		!= from->segs[i]->pts.end()
		)
	      ) ) {
	    chain.push_back(from->segs[i]);
	    if (dbgTrace) cout << "FOUND " << endl;
	    return chain;
	}

	// try to recurse and check result:
	std::vector<const RasterSegment* > c;

	if (dbgTrace) rec++;

	c = getCon(next, to);

	if (dbgTrace) rec--;

	if (c.size()) {
	    chain.push_back( from->segs[i] );
	    for (size_t j=0;j<c.size();j++){
		chain.push_back(c[j]);
	    }
	    return chain;
	} // else try next
    }
    if (dbgTrace) cout << "FOUND end " << chain.size() << endl;
    return chain; // return empty solution
}

// helper function: find connection from a to b (depth first search)
static std::vector< RasterSegment* >  collectSegs (const RasterPoint* from)
{
    using namespace std;

    std::vector< RasterSegment* > chain;
    if (dbgTrace) {
	for (int i=0;i<rec;i++) cout << "\t";
	cout << "C f " << *from << " s=" << from->segs.size() << " v=" << from->visited;
	cout << endl;
    }
    if (from->visited) {
	if (dbgTrace) {
	    for (int i=0;i<rec;i++) cout << "\t";
	    if (from->segs.size()) {
		cout << "FOUND vis! " << *from->segs[0]->otherEnd(from) <<  endl;
	    } else {
		cout << "FOUND vis! " << *from << " SEGS EMPTY! " <<  endl;
	    }
	}
	return chain;
    }
    ((RasterPoint*)from)->visited=true;

    for (size_t i=0;i<from->segs.size();i++) {
	const RasterPoint* next = from->segs[i]->otherEnd(from);

	if (dbgTrace) {
	    for (int k=0;k<rec;k++) cout << "\t";
	    cout << "  ...> " << *next << "\t" 
		 << "\t" << *from << endl;
	}

	// append current
	chain.push_back(from->segs[i]);

	// try to recurse and check result:
	std::vector< RasterSegment* > c;

	if (dbgTrace) rec++;

	c = collectSegs(next);

	if (dbgTrace) rec--;

	if (c.size()) {
	    chain.push_back( from->segs[i] );
	    for (size_t j=0;j<c.size();j++){
		chain.push_back(c[j]);
	    }
	    return chain;
	} // else try next
    }
    if (dbgTrace) cout << "FOUND end " << chain.size() << endl;
    return chain; // return empty solution
}


static void clearVis(std::vector<RasterPoint* > pts)
{
    for (size_t i=0;i<pts.size();i++) {
	pts[i]->visited = false;
    }
}

static void clearVis(std::vector<RasterSegment* > segs)
{
    for (size_t i=0;i<segs.size();i++) {
	segs[i]->start->visited = false;
	segs[i]->end->visited = false;
    }
}

static void getPointSet(std::vector<RasterSegment* >& segs,
			std::vector<RasterPoint*>& points)
{
    clearVis(segs);
    for (size_t i=0;i<segs.size(); i++) {
	RasterSegment* s = segs[i];
	if ( ! s->start->visited ) {
	    points.push_back(s->start);
	    s->start->visited = true;
	}
	if ( ! s->start->visited ) {
	    points.push_back(s->start);
	    s->start->visited = true;
	}
	if ( ! s->end->visited ) {
	    points.push_back(s->end);
	    s->end->visited = true;
	}
    }
}
