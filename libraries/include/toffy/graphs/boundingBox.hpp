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
#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <opencv2/core.hpp>

class BoundingBox
{
   public:
    BoundingBox();
    BoundingBox(int u, int d, int l, int r);
    int up;
    int down;
    int left;
    int right;
    cv::Point getCenter();
    int getArea();
    bool intersectWith(BoundingBox other);
    int DistanceWith(BoundingBox other);
};

#endif  // BOUNDINGBOX_H
