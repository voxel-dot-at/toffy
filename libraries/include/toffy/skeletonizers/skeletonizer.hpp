/*
 * Skeletonizer.h
 *
 *  Created on: Dec 13, 2012
 *      Author: simon
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
