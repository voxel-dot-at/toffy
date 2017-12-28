#pragma once

#include <toffy/filter.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif

namespace toffy {
/**
 * @brief Display a OpenCV Mat image int a windows viewer
 * @ingroup Viewers
 *
 * Takes one single image form the frame and shows is using ImageView from
 * OpenCV.
 *
 * It allows also to scale up or down the image keeping the proportions. Also
 * the image can be convert and display in grayscale
 *
 * \section ex1 Xml Configuration
 * @include imageview.xml
 *
 */
class ImageView : public Filter	{
public:

    static const std::string id_name; ///< Filter identifier
    ImageView();
    virtual ~ImageView();

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    double _scale, ///< Scaled the image keeping the proportions
	_max, ///< Max value for converting to grayscaled
	_min; ///< Min value for converting to grayscaled
    std::string _in_img;
    bool _gray; ///< Flag for converting the image to grayscaled.
    bool _enabled; ///< if enabled, show the image, otherwise skip it
    static std::size_t _filter_counter; ///< Internal filter counter
    cv::Mat show;
};
}
