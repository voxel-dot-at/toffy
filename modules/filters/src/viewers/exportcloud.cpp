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

#include "toffy/viewers/exportcloud.hpp"
#include "toffy/filter_helpers.hpp"

using namespace toffy;
using namespace cv;

void ExportCloud::updateConfig(const boost::property_tree::ptree &pt) {
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
	LOG(debug) << " exporting " << _in_cloud;
#if 0
	pcl::RangeImagePlanar::Ptr planar;
	try {
		planar = boost::any_cast<pcl::RangeImagePlanar::Ptr>(in.getData(_in_cloud));
	} catch(const boost::bad_any_cast &) {
		LOG(warning) <<
			"Could not cast input " << _in_cloud <<
			", filter  " << id() <<" not applied.";
		return false;
	}
#endif
    if (_xyz) {
        return exportXyz(in, out);
    } else {
        return exportPcl2(in, out);
    }
}

bool ExportCloud::exportXyz(const Frame &in, Frame& /*out*/) {
    return true;
}

bool ExportCloud::exportPcl2(const Frame &in, Frame& /*out*/) {
	pcl::PCLPointCloud2::Ptr planar;
	try {
		planar = boost::any_cast<pcl::PCLPointCloud2::Ptr>(in.getData(_in_cloud));
	} catch(const boost::bad_any_cast &) {
		LOG(warning) <<
			"Could not cast input " << _in_cloud <<
			", filter  " << id() <<" not applied.";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        bool success = getInputPoints(in, cloud);
        if (success) {
            planar.reset(new pcl::PCLPointCloud2());
            pcl::toPCLPointCloud2(*cloud, *planar);
        } else {
    		return false;
        }
	}


	std::string fileName = _path +_fileName;
	if (_seq) {
		_cnt++;
		fileName += boost::lexical_cast<std::string>(_cnt);
	}
	fileName += ".pcd";
	_w.write (fileName, *planar);

	return true;
}



bool ExportCloud::getInputPoints(const Frame& in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::RangeImagePlanar::Ptr p;
    matPtr img3d;

    try {
	pcl::PCLPointCloud2Ptr planar;
	planar = boost::any_cast<pcl::PCLPointCloud2Ptr>(in.getData(_in_cloud));
	//     if (*planar != nullptr) {
        pcl::fromPCLPointCloud2(*planar, *cloud);
	//     } else {
	//         LOG(error) << "input cloud is null!";
	//         return false;
	//     }
    } catch(const boost::bad_any_cast &) {
	LOG(warning) <<
	    "Could not cast input - trying to convert from OCV cloud.";

	// 3d mat
	try {
	    matPtr img3d = boost::any_cast<matPtr>(in.getData(_in_cloud));
	    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(/*width=*/img3d->cols, /*height=*/img3d->rows));
	    float *dptr;

	    for (int y=0; y< img3d->rows; y++) {
		for (int x = 0; x < img3d->cols; x++) {
		    dptr = img3d->ptr<float>(y, x);
		    pcl::PointXYZ& pt = cloud->at(x,y);
		    pt.x = dptr[0];
		    pt.y = dptr[1];
		    pt.z = dptr[2];
		}
	    }
	} catch (const boost::bad_any_cast &) {
	    LOG(warning) << "Could not cast input to matPtr ";

	    // try pointcloud:
	    try {
                cloud = boost::any_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr>(in.getData(_in_cloud));
                LOG(info) << "got cloud size " << cloud->size();
                LOG(info) << __LINE__;
	    } catch (const boost::bad_any_cast &) {
                LOG(warning) << "Could not cast input " << _in_cloud << " to PointCloud<pcl::PointXYZ> "
			     << _in_cloud << ", filter  " << id() << " not applied.";


                return false;
	    }
        }
	LOG(debug) << "YUHUU!!!!";
    }
    return true;
}
