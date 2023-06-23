#include <math.h>

#include "toffy/graphs/boundingBox.hpp"

BoundingBox::BoundingBox()
{
}

BoundingBox::BoundingBox(int u, int d, int l, int r)
{
    up = u;
    down = d;
    left = l;
    right = r;
}

cv::Point BoundingBox::getCenter()
{
    int X = right+left /2;
    int Y = up+down /2;
    return cv::Point(X,Y);
}

int BoundingBox::getArea()
{
    return ((right-left)*(down-up));
}

bool BoundingBox::intersectWith(BoundingBox other)
{
    return !(other.left > right
            || other.right < left
            || other.up> down
             || other.down < up);
}

int BoundingBox::DistanceWith(BoundingBox other)
{
    cv::Point A = getCenter();
    cv::Point B = other.getCenter();
    return 0;
}
