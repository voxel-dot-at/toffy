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

#include <opencv2/core/core.hpp>
#include "toffy/skeletonizers/skeletonizer.hpp"
/*
 * K3M Skeletonizer
 * As in the K3M PAPER :
 * K3M : A UNIVERSAL ALGORITHM FOR IMAGE SKELETONIZATION AND A REVIEW OF THINNING TECHNIQUES
 * (http://home.agh.edu.pl/~panasiuk/pl/a/3/k3m.pdf)
 * Implemented by Lambert David
 *
 * Extended by Simon Vogl to be more deterministic as the original 
 * implementation is self-modifying, leading to visual artefacts.
 *
 */
class K3MPPSkeletonizer: public Skeletonizer{
public:
    K3MPPSkeletonizer();
    virtual ~K3MPPSkeletonizer();
    //Main fuction
    //IN (lines): A Grayscale Thresolded picture (0 = BG, Other = Objects)
    //OUT (skel): A Grayscale Skeletonized picture (0 = BG, 1 = Skeleton)
    virtual void skeletonize(const cv::Mat& lines, cv::Mat& skel);
    //virtual void configure(const rapidjson::Value& configObject);
    static Skeletonizer * Create();

private:
    //init A0 => A6 Table
    void init();

    //Mark the border in the Borders Mat
    void markBorder(const cv::Mat& src);

    // inline thinning
    bool phase(cv::Mat in, bool* lookup);

    // thin from in to out
    bool phase(cv::Mat in, cv::Mat& out, bool* lookup);

    // last phase - ignore border
    bool phaseSingle(cv::Mat in, cv::Mat& out, bool* lookup);
    bool phaseSingle(cv::Mat& mat, bool* lookup);

    cv::Mat Borders;

    //Mark the border in the Borders Mat
    void markborder(const cv::Mat& src);

    void computeThickness(const cv::Mat &in);

    //Do a phase if the phase deleted something, return true, els
    //Table that contains true in the weight that need to be removed
    //Initialised by init();
    bool A0[256];
    bool A1[256];
    bool A2[256];
    bool A3[256];
    bool A4[256];
    bool A5[256];
    bool A6[256];
};
