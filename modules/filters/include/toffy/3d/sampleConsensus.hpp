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

/**
 * @brief
 *
 */
namespace toffy {
    namespace filters {
	namespace f3d {
	    const int sampleModelUnknown=0;
	    const int sampleModelLine=1;
	    const int sampleModelPlane=2;

	    /** SampleConsensus analyses clouds with a selectable RANSAC model 
	     */
	    class SampleConsensus : public Filter {
	    public:
		SampleConsensus(): Filter("sampleConsensus"), 
				   in("cloud"),inliers("inliers"), 
				   outliers("outliers"), 
				   model("plane"), threshold(0.02), maxIters(1000) {}

		virtual ~SampleConsensus() {}

		virtual bool filter(const Frame& in, Frame& out);

		virtual int loadConfig(const boost::property_tree::ptree& pt);

		virtual boost::property_tree::ptree getConfig() const;

		void updateConfig(const boost::property_tree::ptree &pt);
	    private:
		std::string in, inliers, outliers, model;
		double threshold;
		int maxIters;
		int mdl; // the numeric value of the model

	    };
	}
    }
}

