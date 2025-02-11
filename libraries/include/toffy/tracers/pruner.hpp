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

#pragma once

#include <opencv2/core.hpp>

#include <toffy/tracers/segmentTracer.hpp>  // for Segments

/**
 *  base class for all skeletonizers
 */

class Pruner
{
   public:
    Pruner();
    virtual ~Pruner();

    /** prune segments - this marks all segments as prunable that end within
	 * the radius of a crossing point. It does not delete segments.
	 */
    virtual void prune(const Segments& in, Segments& out);

    //virtual void configure(const rapidjson::Value& configObject) = 0;

    int slack;  //< constant number of pixels added to the threshold (typ. 0);
    double
        factor;  //< multiplication factor that the thickness is multiplied with (typ 1.0)

    bool
        pruneBorderPoints;  //< keep or prune segments that touch the image border
   protected:
    //Debuging switch
    bool Debugging;
};
