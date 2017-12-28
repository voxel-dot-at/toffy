#pragma once
/*
 * @file bilateral.hpp
 *
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
#include <toffy/filter.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#endif

/**
 * @brief
 *
 */
namespace toffy {
namespace filters {
namespace smoothing {
/** perform bilateral filtering of the (depth) channel.
	 */
class Bilateral : public Filter {
    static std::size_t _filter_counter;
public:
    Bilateral();

    virtual ~Bilateral() {}

    static const std::string id_name;

    virtual bool filter(const Frame& in, Frame& out);

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree &pt);

private:
    std::string _in_img, _out_img;
    double d, sigmaColor, sigmaSpace ;
    cv::Mat _dst;
};
}
}
}


