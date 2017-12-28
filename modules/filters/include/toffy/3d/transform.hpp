#pragma once
/*
 * @file alignment.hpp
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
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif


/**
 * @brief
 *
 */
namespace toffy {
namespace filters {
namespace f3d {

/*class TranformUnit {
    public:
    cv::Vec3d vect;
    int op;
};*/


class Transform : public Filter {

    std::vector<cv::Vec3d> _actions;
    std::vector<int> _operations;
    std::string _in_cloud, _out_cloud;
    static std::size_t _filter_counter;
public:
    Transform();
    virtual ~Transform() {}
    static const std::string id_name;

    virtual bool filter(const Frame& in, Frame& out) const;

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree &pt);

    enum Operation {
	rotation, translation, scaling
    };

};

}}}
