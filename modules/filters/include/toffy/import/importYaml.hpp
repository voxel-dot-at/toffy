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

#include <string>

#include "toffy/filter.hpp"

namespace toffy {
namespace import {

class ImportYaml : public Filter
{
    static std::size_t _filter_counter;
    std::vector<std::pair<std::string, boost::any> > _data;

   public:
    ImportYaml();
    virtual ~ImportYaml() {}

    // virtual int loadConfig(const boost::property_tree::ptree& pt);

    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree& pt);

    virtual bool filter(const Frame& in, Frame& out);

   private:
    std::vector<std::string> fields;  //< the fields of the input frame to save
    std::string _in_cloud, _fileName, _seqName;
    std::string path, prefix;
    bool _seq;
    int start, counter;

    bool loadFromYaml(Frame& f, const std::string& idx);
};

}  // namespace import
}  // namespace toffy
