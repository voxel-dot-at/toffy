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

#include "toffy/filter.hpp"

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#endif



/**
 * @brief
 *
 */
namespace toffy {
    namespace filters {
	namespace f3d {
#if 0
	    /** Split clouds into an inlier and an outlier cloud depending on a defined 3D bounding box
	     */
	    class BBox : public Filter {
		std::string _in_cloud, _out_img;
		float _max_y, _max_x;
		int _scale;
	    public:
		BBox(): Filter("bbox"), _in_cloud("cloud"),
				    _out_img("ground"), _max_y(0), _max_x(0), _scale(100) {}
		virtual ~BBox() {}


		virtual int loadConfig(const cv::FileNode &fn);

		virtual bool filter(const Frame& in, Frame& out);
	    };
#endif

	    /** Split clouds into an inlier and an outlier cloud depending on a defined 3D bounding box.
	     */
	    class Split : public Filter {
	    public:
		Split(): Filter("split"), _in_cloud("cloud"), _out_inliers("inliers"), _out_outliers("outliers"),
			 _axis("z"), _min(0.0), _max(1.0) {}
		virtual ~Split() {}

		virtual bool filter(const Frame& in, Frame& out);

		virtual int loadConfig(const boost::property_tree::ptree& pt);

		//virtual int loadConfig(const cv::FileNode &fn);

		virtual boost::property_tree::ptree getConfig() const;

		void updateConfig(const boost::property_tree::ptree &pt);
	    private:
		std::string _in_cloud, _out_inliers, _out_outliers;
		std::string _axis;
		float _min, _max;
	    };
	}
    }
}
