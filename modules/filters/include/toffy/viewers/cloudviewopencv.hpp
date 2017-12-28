#pragma once

#include "toffy/filter.hpp"
//#include <opencv2/viz.hpp>

namespace toffy {

class CloudViewOpenCv : public Filter	{
    std::string _in_cloud;
    //cv::viz::Viz3d *sameWindow;
public:
    CloudViewOpenCv(): Filter("cloudviewopencv"), _in_cloud("cloud") {
	//sameWindow = new cv::viz::Viz3d();
    }
    virtual ~CloudViewOpenCv(){
	//delete sameWindow;
    }

    virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out) const;
};
}
