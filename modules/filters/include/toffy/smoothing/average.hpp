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
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif

#include <deque>

#ifdef MSVC
#define DLLExport __declspec( dllexport )
#else
#define DLLExport /**/
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
class DLLExport Average : public Filter {
    std::string _in_img, _out_img;
    static std::size_t _filter_counter;
    std::deque<boost::shared_ptr<cv::Mat> > _queue;
    size_t _size;
    cv::Mat _dst, _cnt;
public:
    Average();

    virtual ~Average() {}

    static const std::string id_name;

    //virtual int loadConfig(const boost::property_tree::ptree& pt);

    virtual bool filter(const Frame& in, Frame& out);

    virtual boost::property_tree::ptree getConfig() const;

    void updateConfig(const boost::property_tree::ptree &pt);

    size_t size() const {return _queue.size();}
};
}
}
}


