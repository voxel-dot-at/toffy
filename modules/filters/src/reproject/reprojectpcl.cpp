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
#include <iostream>

#include <opencv2/core.hpp>

#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/transforms.h>

#include <boost/log/trivial.hpp>
#include <boost/any.hpp>

#include "toffy/common/filenodehelper.hpp"

#include "toffy/reproject/reprojectpcl.hpp"

using namespace toffy;
using namespace cv;
using namespace std;

std::size_t ReprojectPCL::_filter_counter = 1;
const std::string ReprojectPCL::id_name = "reprojectpcl";

ReprojectPCL::ReprojectPCL(): Filter(ReprojectPCL::id_name,_filter_counter),
    _in_img("img"), _out_cloud("cloud"), _world(false)
{
    _filter_counter++;
}

void ReprojectPCL::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    boost::optional<const boost::property_tree::ptree& > ocvo = pt.get_child_optional( "options.cameraMatrix" );
    if (ocvo.is_initialized()) {
	if( toffy::commons::checkOCVNone(*ocvo) ) {
	    boost::property_tree::ptree os;
	    os.put_child("cameraMatrix",*ocvo);
	    cv::FileStorage fs = commons::loadOCVnode(os);
	    //Mat val;
	    fs.getFirstTopLevelNode() >> _cameraMatrix;
	    cout << fs.getFirstTopLevelNode().name() << endl;
	    fs.release();
	} else
	    BOOST_LOG_TRIVIAL(debug) << "Node cameraMatrix is not opencv.";
    } else
	BOOST_LOG_TRIVIAL(debug) << "Node options.cameraMatrix not found.";

    _in_img = pt.get<string>("inputs.img",_in_img);
    _in_cameraMatrix = pt.get<string>("inputs.cameraMatrix",_in_cameraMatrix);

    _out_cloud = pt.get<string>("outputs.cloud",_out_cloud);
    _world = pt.get<bool>("options.world", _world);
    _in_transf = pt.get<string>("inputs.transf",_in_transf);


}

boost::property_tree::ptree ReprojectPCL::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();
    pt.put("options.cameraMatrix", _cameraMatrix);

    pt.put("inputs.img", _in_img);
    pt.put("inputs.cameraMatrix", _in_cameraMatrix);
    pt.put("outputs.cloud", _out_cloud);

    pt.put("options.world", _world);
    pt.put("inputs.transf", _in_transf);


    return pt;
}

bool ReprojectPCL::filter(const Frame &in, Frame& out) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__  <<  " " << id();

    toffy::Filter::setLoggingLvl();

    boost::shared_ptr<cv::Mat> img;
    try {
	img = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_in_img));
    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) <<
				      "Could not cast input " << _in_img <<
				      ", filter  " << id() <<" not applied.";
	return false;
    }
    if (!_in_cameraMatrix.empty()) {
	try {
	    _cameraMatrix= boost::any_cast<cv::Mat>(in.getData(_in_cameraMatrix));
	} catch(const boost::bad_any_cast &) {
	    BOOST_LOG_TRIVIAL(warning) <<
					  "Could not read input " << _in_cameraMatrix;
	}
    }
    if (_cameraMatrix.empty()) {
	BOOST_LOG_TRIVIAL(warning) <<
				      "No cameraMatrix data, filter " << id() <<" not applied.";
	return false;
    }

    pcl::RangeImagePlanar::Ptr planar (new pcl::RangeImagePlanar);
    planar->setDepthImage (img->ptr<float>(0),
			   img->cols,img->rows,
			   _cameraMatrix.at<double>(0,2),
			   _cameraMatrix.at<double>(1,2),
			   _cameraMatrix.at<double>(0,0),
			   _cameraMatrix.at<double>(1,1));

    if (_world) {
	boost::shared_ptr<Eigen::Affine3f> world2camera;
	try {
	    //TODO transformation key by parameter
	    world2camera = boost::any_cast<boost::shared_ptr<Eigen::Affine3f> >(in.getData(_in_transf));//"bta12world"));
	    pcl::transformPointCloud(*planar, *planar, world2camera->matrix());
	} catch(const boost::bad_any_cast &) {
	    BOOST_LOG_TRIVIAL(warning) << "No " <<  _in_transf << " transform found";
	}
    }

    pcl::PCLPointCloud2Ptr planarPC2 (new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*planar,*planarPC2);

    out.addData(_out_cloud,planarPC2);

    return true;
}

