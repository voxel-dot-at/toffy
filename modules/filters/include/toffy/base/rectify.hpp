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
 * @brief Apply the lens distorsion parameters rectification to the image
 * @ingroup Filters
 *
 * \section ex1 Xml Configuration
 * @include rectify.xml
 *
 */
class Rectify : public Filter {
public:
    static const std::string id_name; ///< Filter identifier

    Rectify();
    virtual ~Rectify() {}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    std::string _in_img, ///< Name of the input image
	_out_img, ///< Name of the output image
	_in_cameraMatrix,
	_in_distCoeffs;

    cv::Mat _cameraMatrix, ///< Internal use
	_distCoeffs; ///< Internal use

    bool initialized; ///< true if cameraMatrix and distCoeffs have been set up
    cv::Mat map1, map2; ///< the maps for the remap operation.

    static std::size_t _filter_counter; ///< Internal filter counter  
};
}}
