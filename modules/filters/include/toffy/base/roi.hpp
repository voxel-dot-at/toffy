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
 * @brief Region of interest selector and filter
 * @ingroup Filters
 *
 * For detailled information see \ref roi_page description page
 *
 */
class Roi : public Filter {
public:
    static const std::string id_name; ///< Filter identifier

    Roi();
    virtual ~Roi() {}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    std::string _in_img, ///< Input image name
	_out_img; ///< Output filtered image name
    double _x, ///< Start x coordinate of the roi
	_y,  ///< Start y coordinate of the roi
	_width, ///< Horizontal size of the roi from _x
	_height, ///< vertical size of the roi from _y
	_inValue, ///< Value to be check against the input data of the roi
	_outValue; ///< Value to be set in the filtered pixels.
    bool _in, /**< Flag indicating if we should check and filter the inner part of the roi or the outside*/
	_filter, ///< Flag to activate filtering by _inValue and _below flag
	_below; /**< Flag to indicate if the values over of bellow the _inValue
		well be check */
    static std::size_t _filter_counter; ///< Internal filter counter
    cv::Rect _roi; ///< OpenCV rectangle defining the roi
};
}}
