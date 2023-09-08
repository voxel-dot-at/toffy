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
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core.hpp>

#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/any.hpp>

#include "toffy/filter_helpers.hpp"
#include "toffy/viewers/exportYaml.hpp"

using namespace toffy;
using namespace cv;

void ExportYaml::updateConfig(const boost::property_tree::ptree &pt) {
    LOG(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _fileName = pt.get("options.fileName",_fileName);
    _path = pt.get("options.path",_path);
    _seq = pt.get<bool>("options.sequence",_seq);
    _bin = pt.get<bool>("options.binary",_bin);

    pt_optional_get_default( pt, "options.xyz",_xyz, false);

    _in_cloud = pt.get<std::string>("inputs.cloud",_in_cloud);
}

boost::property_tree::ptree ExportYaml::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.fileName", _fileName);
    pt.put("options.path", _path);
    pt.put("options.sequence", _seq);
    pt.put("options.binary", _bin);

    pt.put("inputs.cloud", _in_cloud);

    return pt;
}

bool ExportYaml::filter(const Frame &in, Frame& out) {
	LOG(debug) << " exporting " << _in_cloud;
    return true;
}
