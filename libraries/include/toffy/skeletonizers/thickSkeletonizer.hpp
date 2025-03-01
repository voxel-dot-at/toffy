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
 *	First Attempt of thinning.
 *	Sometime produce Artifacts.
 *	Used by K3MSkeletonizer for the last pass.
 */
#ifndef THICKSKELETONIZER_H_
#define THICKSKELETONIZER_H_

#include <opencv2/core/core.hpp>
#include "toffy/skeletonizers/skeletonizer.hpp"
#include "toffy/skeletonizers/erodeSkeletonizer.hpp"

class ThickSkeletonizer: public Skeletonizer{
public:
    ThickSkeletonizer();
    virtual ~ThickSkeletonizer();



    virtual void skeletonize(const cv::Mat& lines, cv::Mat& skel);

    /** in: a binary image (!=0 == region); out: computed
     *
     */
    virtual void computeThickness(const cv::Mat& in, cv::Mat& thick);
    //virtual void configure(const rapidjson::Value& configObject);
    int maxT;

    cv::Mat max; ///< store local maxima
    cv::Mat skeleton_1st_pass; ///< store thickness information
    cv::Mat skeleton_last_pass; ///< store thickness information
    cv::Mat skeleton_merged; ///< store thickness information

    static Skeletonizer * Create();
protected:
    /** thinning by morphological operations; reads a 8UC1 picture and writes one.
     *
     */
    virtual void skel(const cv::Mat& in, cv::Mat& skel);

    // @return the number of pixels removed
    int skel_once(const cv::Mat& in, cv::Mat& skel); ///< last go..


    /** find all points with val >= max(neighbours), input: 8UC1 out 8UC1, [0,1]
     *
     */
    virtual void findLocalMaxima(const cv::Mat& in, cv::Mat& thick);


    virtual void mergeMax(const cv::Mat& thick, cv::Mat& in);

    struct action {
        unsigned char code;
        bool isKnown; // check if we have created this code yet.
        bool setPixel; // set or keep this pixel.
    };

    action actions[256];
    action skels[256];

    void setAction(action actions[], unsigned char code, bool set);
    void setDiagRotatedActions(action actions[], unsigned char code, bool set);
    void set90DegRotatedActions(action actions[], unsigned char code, bool set);

    void initActions();
};

#endif //THICKSKELETONIZER
