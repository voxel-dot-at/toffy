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

class OffSet : public Filter {

    std::vector<cv::Rect> _rois;
    double _sumValue, _mulValue;
    std::string _in_img, _out_img;
    static std::size_t _filter_counter;
public:
    OffSet();
    virtual ~OffSet() {}
    static const std::string id_name;

    void addRoi(cv::Rect newRoi);

    //virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);


    virtual bool filter(const Frame& in, Frame& out) const;

};

}
}
