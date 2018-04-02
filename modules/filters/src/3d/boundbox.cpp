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
#include <pcl/conversions.h>

#include <toffy/3d/boundbox.hpp>

//using namespace toffy;

void toffy::get_bound_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
			 struct bound_box *bb)
{
    toffy::get_bound_box(*cloud, bb);
}

void toffy::get_bound_box(pcl::PointCloud<pcl::PointXYZ>& cloud,
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

#ifdef PCL_VISUALIZATION
void toffy::get_bound_box(pcl::RangeImagePlanar::Ptr cloud,
			 struct bound_box *bb)
{
    toffy::get_bound_box(*cloud, bb);
}

void toffy::get_bound_box(pcl::RangeImagePlanar& cloud,
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
#endif
void toffy::get_bound_box(pcl::PCLPointCloud2Ptr cloud,
			 struct bound_box *bb)
{
    toffy::get_bound_box(*cloud, bb);
}

void toffy::get_bound_box(pcl::PCLPointCloud2& cloud,
			 struct bound_box *bb)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud,*cloudXYZ);
    toffy::get_bound_box(cloudXYZ, bb);
    return;
}

