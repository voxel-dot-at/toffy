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
 *	Thinning using Erosion.
 */
#ifndef ERODESKELETONIZER_H_
#define ERODESKELETONIZER_H_

#include <opencv2/core/core.hpp>
#include "toffy/skeletonizers/skeletonizer.hpp"

class ErodeSkeletonizer: public Skeletonizer{
public:
	ErodeSkeletonizer();
	virtual ~ErodeSkeletonizer();

	virtual void skeletonize(const cv::Mat& lines, cv::Mat& skel);
    //virtual void configure(const rapidjson::Value& configObject);
    static Skeletonizer*  Create();
protected:
	virtual void skel(const cv::Mat& in, cv::Mat& skel);

};

#endif
