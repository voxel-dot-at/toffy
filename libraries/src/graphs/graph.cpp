/*
 * Graph.cpp
 *
 *  Created on: Jan 17, 2013
 *      Author: simon
 */
#include "toffy/graphs/graph.hpp"

#if OCV_VERSION_MAJOR >= 3
#include <opencv2/core.hpp>
#else
#include <opencv2/core/core.hpp>
#endif

int Graph::count=0;


BoundingBox Graph::getBoundingBox()
{
    std::vector<cv::Point*> tmpPoints;
    points(tmpPoints);
    int u,d,l,r;
    u = tmpPoints[0]->y;
    d = tmpPoints[0]->y;
    l = tmpPoints[0]->x;
    r = tmpPoints[0]->x;
    for(int j = 1; j < tmpPoints.size();j++){
        if(tmpPoints[j]->y < u){
            u = tmpPoints[j]->y;
        }
        else if(tmpPoints[j]->y > d){
            d = tmpPoints[j]->y;
        }
        if(tmpPoints[j]->x < l){
            l = tmpPoints[j]->x;
        }
        else if(tmpPoints[j]->x > r){
            r = tmpPoints[j]->x;
        }
    }
    BoundingBox tmpBbox;
    tmpBbox.up = u - ((d-u) / 1.25);
    tmpBbox.down = d+ ((d-u) / 1.25);
    tmpBbox.left = l- ((r-l) / 1.25);
    tmpBbox.right = r+ ((r-l) / 1.25);

    return tmpBbox;
}
