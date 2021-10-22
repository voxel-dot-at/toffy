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

#include <pcl/point_cloud.h>

#include "toffy/filter.hpp"

/**
 * @brief SampleConsensus implements access to ransac filters. It reads in a 3D pointset (matPtr) and outputs an inliers and an outliers cloud.
 * Depending on the sampling mode (line, plane, cylinder), type-specific options are available. 
 */
namespace toffy {
    namespace filters {
	namespace f3d {
            enum sampleType { unknown, line, plane, cylinder, regionGrowing, euclidean, mincut };
            static std::string sampleTypeNames[] = { "unknown", "line", "plane", "cylinder", "regionGrowing", "euclidean", "min-cut" };

	    /** SampleConsensus analyses clouds with a selectable RANSAC model 
	     */
	    class SampleConsensus : public Filter {
	    public:
		SampleConsensus(): Filter("sampleConsensus"), 
            in("cloud"),inliers("inliers"), 
            outliers("outliers"), 
            model("plane"), threshold(0.02), 
            minRadius(0.05), maxRadius(0.3),
            maxIters(1000), type(plane),
            outputPC2(false),
            smoothnessThresh(5.0),curvatureThresh(1.0),
            distanceThresh(0.05),
            minSize(100), maxSize(25000)
         {}

		virtual ~SampleConsensus() {}

		virtual boost::property_tree::ptree getConfig() const;

		void updateConfig(const boost::property_tree::ptree &pt);

		virtual bool filter(const Frame& in, Frame& out);
	    private:
                bool getInputPoints(const Frame& in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
                bool segmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers);
                bool segmentCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& inliers, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& outliers);

                bool runRegionGrowing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        Frame& out);
                bool runEuclidean(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        Frame& out);
                bool runEuclideanOld(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        Frame& out);
                bool runMinCut(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        Frame& out);

		std::string in, inliers, outliers, model;
		double threshold, minRadius,maxRadius;
		int maxIters;
		sampleType type;
        bool outputPC2; ///< output PointCloud2 (true) or PointCloud<PointXYZ> (false)

        // regionGrowing wants:
        double smoothnessThresh; ///< smoothness threshold 
        double curvatureThresh; ///< curvature threshold
        
        // euclidean clustering:
        double distanceThresh; ///< distance threshold (meters)
        int minSize, maxSize;
	    };
	}
    }
}
