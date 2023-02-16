/*
   Copyright 2023 Simon Vogl <simon@voxel.at>

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

namespace toffy {
    namespace filters {
	namespace f3d {

        /**
         * @brief converts x,y,z planes as delivered by the Becom Systems Toreo camera into a PointCloud::Ptr .
         * Depending if amplitudes is set, it creates a PointXYZ or PointXYZA cloud.
         *
         */
	    class Xyz2Pcl : public Filter {
	    public:
            Xyz2Pcl(): Filter("xyz2pcl"),out_cloud("cloud"),min(100), max(3000), amplitudes(false), pcl2(false) {}
            virtual ~Xyz2Pcl() {}

            void updateConfig(const boost::property_tree::ptree &pt);

            virtual boost::property_tree::ptree getConfig() const;

            virtual bool filter(const Frame& in, Frame& out);

	    private:
            std::string out_cloud;
            int min, max; ///< min,max values for amplitude scaling
            bool amplitudes;
            bool pcl2;

            //  border limits, pixel:
            int borderLeft, borderRight;
            int borderTop, borderBottom;

            bool convertXyz(const Frame& in, Frame& out, toffy::matPtr x,toffy::matPtr y,toffy::matPtr z);
            bool convertXyzA(const Frame& in, Frame& out, toffy::matPtr x,toffy::matPtr y,toffy::matPtr z,toffy::matPtr ampl);
	    };
	}
    }
}
