#pragma once

#include "toffy/filter.hpp"
#include <pcl/io/pcd_io.h>

namespace toffy {

class ExportCloud : public Filter	{
    std::string _in_cloud, _path, _fileName;
    bool _seq, _bin;
    pcl::PCDWriter _w;
    int _cnt;
public:
    ExportCloud(): Filter("exportcloud"),
	_in_cloud("cloud"), _fileName("cloud"),
	_seq(false), _cnt(1) {}
    virtual ~ExportCloud() {}

    virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);
};
}
