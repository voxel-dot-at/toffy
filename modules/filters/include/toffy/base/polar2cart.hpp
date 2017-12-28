#pragma once

#include <toffy/filter.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif

namespace toffy {
namespace filters {
/**
 * @brief Transform depth images distances from polar to cartesian coordiantes
 * using the camera lens fov
 * @ingroup Filters
 *
 * \section ex1 Xml Configuration
 * @include polar2cart.xml
 *
 */
class Polar2Cart : public Filter {
public:

    static const std::string id_name; ///< Filter identifier
    Polar2Cart();
    virtual ~Polar2Cart() {}

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    double _fovx, _fovy;
    std::string _in_img,
	_out_img,
	_in_fovx,
	_in_fovy;
    cv::Mat _cameraMatrix;
    static std::size_t _filter_counter;
};
}}
