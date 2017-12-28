#pragma once

#include <toffy/filter.hpp>
#include "opencv2/video/tracking.hpp"

namespace toffy {
namespace filters {
namespace smoothing {
    class KalmanAverage: public toffy::Filter
    {
	std::string _in_img, out_img, _in_mask;

	float processNoiseCov;     //< process noise covariance value (1e-5)
	float measurementNoiseCov; //< measurement noise covar value  (1e-1)
	bool skipZeros;            //< skip zero distance values from averaging

	boost::shared_ptr<toffy::Filter> _avg;
	boost::shared_ptr<cv::Mat> avgdImg;
	boost::shared_ptr<std::vector<cv::KalmanFilter > > vKF;
	static std::size_t _filter_counter;
    public:
	KalmanAverage();
	virtual ~KalmanAverage();

	static const std::string id_name;
	/** call the filter functions in the filter bank
	 */
	virtual bool filter(const toffy::Frame& in, toffy::Frame& out);

	//virtual int loadConfig(const boost::property_tree::ptree& pt);
	virtual boost::property_tree::ptree getConfig() const;
	virtual void updateConfig(const boost::property_tree::ptree &pt);

	static Filter* creator() {return new KalmanAverage();}
	static Filter* kalmanAverageCreator() {return new KalmanAverage();}
    };
    }
    }
}
