#pragma once

#include <opencv2/core.hpp>

#include "toffy/filter.hpp"

namespace toffy {

/**
 * @brief Reproject a depth image into a 3D cloud using PCL
 * @ingroup Filters
 *
 * Takes a depth image and the camera matrix to convert the distances
 * into a 3D coordinates.
 * It uses the RangeImagePlanar from PCL
 *
 * We can also get the cloud in wcs using the camera pose (or other
 * transformation available).
 *
 * The output in the new PCL::PointCloud2 format.
 *
 * \section Xml Configuration
 * @include reprojectpcl.xml
 *
 */
class ReprojectPCL : public Filter	{
public:

    static const std::string id_name; ///< Filter identifier
    ReprojectPCL();
    virtual ~ReprojectPCL(){}

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    cv::Mat _cameraMatrix; ///< internal camera matrix var
    std::string _in_img, ///< Name of the input depth image
	_in_cameraMatrix, ///< Name of the input camera matrix
	_out_cloud, ///< Name of the output cloud
	_in_transf; ///< Name of the input transformation
    bool _world; ///< Flag to transform the cloud from camera to _in_transf cs
    static std::size_t _filter_counter; ///< Internal filter counter
};
}
