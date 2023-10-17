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

namespace toffy {

/**
 * @brief export fields in frame in to yaml files. filed names are specified in a list of strings in option.fields.field
 * 
 */
class ExportYaml : public Filter	{
public:
    ExportYaml(): Filter("exportYaml"),
	_in_cloud("cloud"),_seqName(""), path("./"), prefix("tofData"),
        useCounter(true), useFc(false),
	counter(1) {}
    virtual ~ExportYaml() {}

    virtual boost::property_tree::ptree getConfig() const;

    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    std::string _in_cloud, _fileName, _seqName;
    std::string path, prefix;
    bool useCounter, useFc;
    pcl::PCDWriter _w;
    int counter;

    void dumpToYaml(const Frame& frame, const std::string& id);
};
}
