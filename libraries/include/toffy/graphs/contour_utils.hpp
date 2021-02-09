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
#include <boost/math/special_functions/fpclassify.hpp>
/**
 * boundary handling utilities
 *
 */

/** find the point on a contour with minimal distance to p; if dist==0 we stop as it is an exact match. 
 * @return the index of the point with minimal distance or -1 if contour is empty
 */
static int findNearestContourPt(const std::vector<cv::Point>& contour, cv::Point2f p, float& dist)
{
    int minIdx=-1;
    float minDist=10000.f;
    for (size_t i=0 ; i<contour.size() ; i++) {
	const cv::Point& q = contour[i];
	float d = (p.x-q.x)*(p.x-q.x) 
	    + (p.y-q.y)*(p.y-q.y);
	if (d<minDist) {
	    minIdx = i;
	    minDist = d;

	    if (d==0) // there's only one exact match!
		break;
	}
    }
    dist = sqrt(minDist);
    return minIdx;
}

/** compute a freeman-based chain code from a->b. 
 * directions look like this:
 * <pre>
 * 3 2 1
 * 4 * 0
 * 5 6 7
 * </pre>
 */
static inline void getAngle(const cv::Point&a, const cv::Point&b, int& angle, float& dist) 
{
    int dx = b.x - a.x;
    int dy = b.y - a.y;
    //cout << cv::Point(dx, dy) << " ";
    
    if ( dx > 0) {
	if (dy>0) {
	    dist=1.41;angle=1;
	} else if (dy<0) {
	    dist=1.41;angle=7;
	} else {
	    dist=1   ;angle=0;
	}
    } else if (dx < 0) {
	if (dy>0) {
	    dist=1.41;angle=3;
	} else if (dy<0) {
	    dist=1.41;angle=5;
	} else {
	    dist=1   ;angle=4;
	}
    } else { // dx==0
	if (dy>0) {
	    dist=   1;angle=2;
	} else if (dy<0) {
	    dist=   1;angle=6;
	} else {
	    dist=   0;angle=0; // error..?!
	}
    }

}

/** compute relative change (1 .. 45deg ccw, -1 45deg clockwise turn etc.)
 * @TODO - needs fixing.
 */
/*static int relaAngle(int old, int cur)
{
    int da = cur - old;
    //cout << "[" << da << "]";
    if ( da < 0 )
	da = 8+da; // TODO fixme
    return da;
}*/

/** compute the angle (normed dot product of vecA o vecB )
 */
static inline double vecAngle(cv::Vec2f vecA,cv::Vec2f vecB)
{
    vecA/=norm(vecA);
    vecB/=norm(vecB);
    return vecA.dot(vecB);
}

/** compute an angle between 3 points - (idx-d) - (idx) - (idx+d) ; return the cosine of the angle
 */
static bool computeTripleAngle(const std::vector<cv::Point>& contour, int idx, int distance, float& cosAngle)
{
    int idxA = idx-distance;  if (idxA<0) idxA+=contour.size();
    int idxC = idx+distance;  if (idxC>=(int)contour.size() ) idxC-=contour.size();
    cv::Point2f a = contour[(unsigned int)idxA];
    cv::Point2f b = contour[(unsigned int)idx];
    cv::Point2f c = contour[(unsigned int)idxC];
    cv::Vec2f ba = b-a;
    cv::Vec2f bc = b-c;
    
    ba /= norm(ba);
    bc /= norm(bc);
    cosAngle = ba.dot(bc);
    return true;
}

/** init a matrix for plotting curvature data
 */
static void initCurv(cv::Mat& m, const std::vector<cv::Point>& contour)
{
    m = cv::Mat(cv::Size(contour.size(), 200), CV_8UC3);
    m=cv::Scalar(255,255,255);
    // zero line
    line(m, cv::Point(0,100), cv::Point(contour.size()-1,100), cv::Scalar(127,127,127));
}

/** print curvature (angle) computed using three points at a fixed distance.
 * +1 is at the image top; -1 is at the image bottom.
 */
static void printCurv(cv::Mat& m, cv::Scalar color, const std::vector<cv::Point>& contour, int distance)
{
    float cos;
    computeTripleAngle(contour, 0, distance, cos);
    int lastY=100 - cos*100;
    for (size_t i=1;i<contour.size();i++) {
	computeTripleAngle(contour, i, distance, cos);
	int y = 100 - cos*100;
	if (!boost::math::isnan(y) && !boost::math::isnan(lastY)) {
	    line(m, cv::Point(i-1,lastY), cv::Point(i,y), color);
	    lastY=y;
	}
    }
}
