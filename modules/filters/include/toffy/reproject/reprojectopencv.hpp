#pragma once

#include "toffy/filter.hpp"

namespace toffy {

/**
 * @brief Reproject a depth image into a 3D cloud using OpenCV
 * @ingroup Filters
 *
 * Takes a depth image and the camera matrix to convert the distances
 * into a 3D coordinates.
 *
 * @todo We can also get the cloud in wcs using the camera pose (or other
 * transformation available).
 *
 * The output is a OpenCV Mat of CV_32FC3 values.
 *
 * \section Xml Configuration
 * @include reprojectopencv.xml
 *
 */
class ReprojectOpenCv : public Filter {

public:

    static const std::string id_name; ///< Filter identifier
    ReprojectOpenCv();
    virtual ~ReprojectOpenCv() {}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);
private:
    cv::Mat _cameraMatrix; ///< internal camera matrix var
    std::string _in_img, ///< Name of the input depth image
	_in_cameraMatrix, ///< Name of the input camera matrix
	_out_cloud; ///< Name of the output cloud
    static std::size_t _filter_counter; ///< Internal filter counter
};
}
