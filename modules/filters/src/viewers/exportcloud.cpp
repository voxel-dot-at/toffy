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
#include <boost/any.hpp>

#include "toffy/viewers/exportcloud.hpp"


using namespace toffy;
using namespace cv;

/*
int ExportCloud::loadConfig(const FileNode &fn) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

	FileNode expCloud = fn;
	if (!checkFileNode(expCloud))
		return 0;

	expCloud["fileName"] >> _name;
	expCloud["path"] >> _path;
	expCloud["sequence"] >> _seq;
	expCloud["binary"] >> _bin;

	//Look for inputs
	FileNode ios = expCloud["inputs"];
	if (ios.empty()) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Missing inputs for filter " <<	id() <<
			" ... using defauts: " <<
			"\tcloud: " << _in_cloud;
	} else {
		ios["cloud"] >> _in_cloud;
	}

	std::string name;
	expCloud["name"] >> name;
	if (!name.empty())
		this->name(name);

	return 1;
}
*/

int ExportCloud::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const boost::property_tree::ptree& pt)";
    const boost::property_tree::ptree& rectify = pt.get_child(this->type());

    loadGlobals(rectify);

    updateConfig(rectify);

    return true;
}

void ExportCloud::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _fileName = pt.get("options.fileName",_fileName);
    _path = pt.get("options.path",_path);
    _seq = pt.get<bool>("options.sequence",_seq);
    _bin = pt.get<bool>("options.binary",_bin);

    _in_cloud = pt.get<std::string>("inputs.cloud",_in_cloud);
}

boost::property_tree::ptree ExportCloud::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.fileName", _fileName);
    pt.put("options.path", _path);
    pt.put("options.sequence", _seq);
    pt.put("options.binary", _bin);

    pt.put("inputs.cloud", _in_cloud);

    return pt;
}

bool ExportCloud::filter(const Frame &in, Frame& out) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

	pcl::RangeImagePlanar::Ptr planar;
	try {
		planar = boost::any_cast<pcl::RangeImagePlanar::Ptr>(in.getData(_in_cloud));
	} catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Could not cast input " << _in_cloud <<
			", filter  " << id() <<" not applied.";
		return false;
	}

	std::string fileName = _path +_fileName;
	if (_seq) {
		_cnt++;
		fileName += boost::lexical_cast<std::string>(_cnt);
	}
	fileName += ".pcd";
	_w.write (fileName, *planar, _bin);

	return true;
}

