#pragma once
/*
 * Copyright (c) 2015
 * VoXel Interaction Design GmbH
 *
 * VoXel.at <office@voxel.at>
 * @author Angel Merino-Sastre <amerino@voxel.at>
 * @date 4 Sep 2014
 * @version 1.0-rc2
 *
 * @brief
 *
 * Description
 *
 *
 * Usage:
 */

#include "toffy/filter.hpp"

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#endif



/**
 * @brief
 *
 */
namespace toffy	    {
 namespace filters   {
  namespace f3d	      {

class GroundProjection : public Filter {
public:

    static const std::string id_name;
    GroundProjection();
    virtual ~GroundProjection() {}

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    std::string _in_cloud, _in_img, _out_img;
    float _max_y, _max_x;
    int _scale;
    double _fovx, _fovy, _dis;
    bool _projBack;
    cv::Mat _cameraMatrix;
    static std::size_t _filter_counter;
};

}}}
