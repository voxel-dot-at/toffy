/*
   Copyright 2018 Simon Vogl <svogl@voxel.at>
                  Angel Merino-Sastre <amerino@voxel.at>

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

#include <cfloat>
#include <iostream>
#include "toffy/toffy_config.h"

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#ifdef PCL_VISUALIZATION
#include <pcl/range_image/range_image_planar.h>
#endif

namespace toffy {

struct bound_box {
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double zmin;
    double zmax;

    bound_box() :xmin(0), xmax(0), ymin(0), ymax(0), zmin(0), zmax(0) {}
    bound_box(const struct bound_box& b) : xmin(b.xmin), xmax(b.xmax),
	ymin(b.ymin), ymax(b.ymax),
	zmin(b.zmin), zmax(b.zmin)
    {}

    bool inside(const pcl::PointXYZ& p) const {
	if ( p.x < xmin || p.x > xmax )
	    return false;
	if ( p.y < ymin || p.y > ymax )
	    return false;
	if ( p.z < zmin || p.z > zmax )
	    return false;
	return true;
    }

    bool inside(const cv::Point3f& p) const {
	if ( p.x < xmin || p.x > xmax )
	    return false;
	if ( p.y < ymin || p.y > ymax )
	    return false;
	if ( p.z < zmin || p.z > zmax )
	    return false;
	return true;
    }


    friend std::ostream& operator<<(std::ostream& o, const struct bound_box& b) {
	o << "[[" << b.xmin << " - " << b.xmax << " , "
	  << b.ymin << " - " << b.ymax << " , "
	  << b.zmin << " - " << b.zmax << "]]";
	return o; }
};


template<typename P>
void get_bound_box(typename pcl::PointCloud<P>& cloud,
		       struct bound_box *bb)
{
    bb->xmax = bb->ymax = bb->zmax = - DBL_MAX;
    bb->xmin = bb->ymin = bb->zmin = DBL_MAX;

    // calculate max and min for every dimension
    int size = cloud.size();
    for (int i = 0; i < size; i++) {
	if (cloud.points[i].x > bb->xmax)
	    bb->xmax = cloud.points[i].x;
	if (cloud.points[i].x < bb->xmin)
	    bb->xmin = cloud.points[i].x;
	if (cloud.points[i].y > bb->ymax)
	    bb->ymax = cloud.points[i].y;
	if (cloud.points[i].y < bb->ymin)
	    bb->ymin = cloud.points[i].y;
	if (cloud.points[i].z > bb->zmax)
	    bb->zmax = cloud.points[i].z;
	if (cloud.points[i].z < bb->zmin)
	    bb->zmin = cloud.points[i].z;
    }
    return;
}

template<typename P>
void get_bound_box(typename pcl::PointCloud<P>::Ptr cloud,
		       struct bound_box *bb)
{
    get_bound_box(*cloud, bb);
}


void get_bound_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
		       struct bound_box *bb);
void get_bound_box(pcl::PointCloud<pcl::PointXYZ>& cloud,
		       struct bound_box *bb);


#ifdef PCL_VISUALIZATION
void get_bound_box(pcl::RangeImagePlanar::Ptr cloud,
		       struct bound_box *bb);
void get_bound_box(pcl::RangeImagePlanar& cloud,
		       struct bound_box *bb);
#endif


void get_bound_box(pcl::PCLPointCloud2& cloud,
		       struct bound_box *bb);
void get_bound_box(pcl::PCLPointCloud2Ptr cloud,
		       struct bound_box *bb);

}
