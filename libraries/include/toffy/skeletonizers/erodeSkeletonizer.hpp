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
