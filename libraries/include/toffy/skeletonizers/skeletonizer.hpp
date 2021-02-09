/*
   Copyright 2012-2021 VoXel Interaction Design - www.voxel.at

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

#ifndef SKELETONIZER_H_
#define SKELETONIZER_H_

#include <opencv2/core/core.hpp>
//#include "rapidjson/document.h"

/**
 *  base class for all skeletonizers
 */


class Skeletonizer {
 public:
    Skeletonizer();
    virtual ~Skeletonizer();

    /** skeletonize a single plane image - fg is >0, bg is 0
     * sub-classes return a skeleton with fg>0, skeletonized, possibly leaving pixels
     * in place.
     *
     * the operation does not work in-place.
     *
     */
    virtual void skeletonize(const cv::Mat& lines, cv::Mat& skel) = 0;
    //virtual void configure(const rapidjson::Value& configObject) = 0;
    cv::Mat thickness;
    cv::Mat Input;
 protected:
    //Debuging switch
    bool Debugging;
};




#endif /* SKELETONIZER_H_ */
