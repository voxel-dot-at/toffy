/*
   Copyright 2018 Simon Vogl <svogl@voxel.at>
                  Angel Merino-Sastre <amerino@voxel.at>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "toffy/filter.hpp"

namespace toffy {

class ExportCloud : public Filter	{
public:
    ExportCloud(): Filter("exportcloud"),
	_in_cloud("cloud"), _fileName("cloud"),
        _pattern(""),_seqName(""),
	_seq(false), _bin(true), _cnt(1) {}
    virtual ~ExportCloud() {}

    virtual boost::property_tree::ptree getConfig() const;

    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    std::string _in_cloud, _path, _fileName, _pattern, _seqName;
    bool _seq, _bin, _xyz;
    pcl::PCDWriter _w;
    int _cnt;

    bool getInputPoints(const Frame& in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    bool exportPcl2(const Frame& in, Frame& out);
    bool exportXyz(const Frame& in, Frame& out);

};
}
