#pragma once

#include <vector>
#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/container/flat_map.hpp>

#include <toffy/frame.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif



namespace toffy
{
    typedef boost::shared_ptr<cv::Mat> matPtr;

    const std::string btaMf = "mf"; ///< constant for modulation frequency
    const std::string btaIt = "it"; ///< constant for integration time
    const std::string btaFc = "fc"; ///< constant for frame counter

    const std::string btaDepth = "depth"; ///< constant for depth mat
    const std::string btaAmpl = "ampl"; ///< constant for amplitudes mat

    class TOFFY_EXPORT BtaFrame : public Frame {

    public:
	BtaFrame() {}
	BtaFrame(const Frame& f): Frame(f) {}

	virtual ~BtaFrame() {}

	matPtr getDepth(const std::string& key = btaDepth) {
	    return getMatPtr(key);
	}
    };
}
