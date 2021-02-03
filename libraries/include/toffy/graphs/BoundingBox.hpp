#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#if OCV_VERSION_MAJOR >= 3
#include <opencv2/core.hpp>
#else
#include <opencv2/core/core.hpp>
#endif

class BoundingBox
{
public:
    BoundingBox();
    BoundingBox(int u,int d,int l, int r);
    int up;
    int down;
    int left;
    int right;
    cv::Point getCenter();
    int getArea();
    bool intersectWith(BoundingBox other);
    int DistanceWith(BoundingBox other);
};

#endif // BOUNDINGBOX_H
