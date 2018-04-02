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
