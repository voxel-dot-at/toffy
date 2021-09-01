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
#include <opencv2/core.hpp>

#include <boost/any.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/log/trivial.hpp>

#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PCLPointCloud2.h>

#if defined(MSVC)
#  include <Windows.h>
#endif

#include "toffy/viewers/cloudviewpcl.hpp"

using namespace toffy;
using namespace cv;

std::size_t CloudViewPCL::_filter_counter = 1;
const std::string CloudViewPCL::id_name = "cloudviewpcl";

CloudViewPCL::CloudViewPCL(): Filter(CloudViewPCL::id_name,_filter_counter),
    _in_cloud("cloud"), _signal(0), _coordSize(4.0),
    _planar(new pcl::PCLPointCloud2())
{
    _filter_counter++;
}

CloudViewPCL::~CloudViewPCL()
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    _signal = 0;
    _thread.join();
    // Vtk does not close the windows in linux :-(

}

boost::property_tree::ptree CloudViewPCL::getConfig() const
{
    boost::property_tree::ptree pt;

    pt.put("options.coordSize", _coordSize);

    for (size_t i=0; i < _clouds.size(); i++) {
	//std::string key="inputs."+= cloudNames[i];
	//key += cloudNames[i];
	pt.put("inputs."+ _cloudNames[i], _cloudNames[i]);
    }

    return pt;
}

void CloudViewPCL::updateConfig(const boost::property_tree::ptree &pt)
{
    using namespace boost::property_tree;

    Filter::updateConfig(pt);
    _coordSize = pt.get("options.coordSize", _coordSize);

    const ptree inputs = pt.get_child("inputs");
    ptree::const_iterator it = inputs.begin();

    _cloudNames.clear();
    _clouds.clear();
    while (it != inputs.end()) {

	if (it->first != "<xmlcomment>") {
	    _cloudNames.push_back( it->second.data() );
        cout << "CloudViewPCL::updateConfig() Listening for cloud " << it->second.data() << endl;
	    pcl::PCLPointCloud2Ptr p(new pcl::PCLPointCloud2());
	    _clouds.push_back( p );
	}
	it++;
    }
    _newCloud=true;

}


void CloudViewPCL::loopViewer()
{
    const std::string ref="reference"; // default id
    std::vector<pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr > hdlr;

    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    pcl::visualization::PCLVisualizer viewer(std::string("3D Viewer ")+id() + " " +_in_cloud);
#if (PCL_MAJOR_VERSION == 1) && (PCL_MINOR_VERSION >= 7)
    viewer.addCoordinateSystem(_coordSize, ref);
#else
    viewer.addCoordinateSystem(_coordSize);
#endif
    for (size_t i=0;i<_clouds.size();i++) {
	double r, g, b;
	pcl::visualization::getRandomColors (r, g, b );
	pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr colorH;
	//colorH.reset( new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2> (
	//		  _clouds[i]/*, (int)(r*256), (int)(g*256), (int)(b*256) */));
	colorH.reset( new pcl::visualization::PointCloudColorHandlerCustom
		      <pcl::PCLPointCloud2> ( _clouds[i],
					      (int)(r*256),
					      (int)(g*256),
					      (int)(b*256)));

	//pcl::PCLPointCloud2ConstPtr cld ( _clouds[i] );
	hdlr.push_back(colorH);
	viewer.addPointCloud(_clouds[i], colorH,
			     Eigen::Vector4f (0.0f, 0.0f, 0.0f, 0.0f),
			     Eigen::Quaternionf (1.0f, 0.0f, 0.0f, 0.0f),
			     _cloudNames[i]);
    }


    while(!viewer.wasStopped() && _signal) {
	//	clouds[0]->getMinMaxRanges (min, max);

	if (_newCloud) {
#if (PCL_MAJOR_VERSION == 1) && (PCL_MINOR_VERSION >= 7)
	    viewer.removeCoordinateSystem(ref);
	    viewer.addCoordinateSystem(_coordSize, ref);
#else
  viewer.removeCoordinateSystem();
  viewer.addCoordinateSystem(_coordSize);
#endif
	    /*
	if (!_cube.empty()) {
	    viewer.removeShape("cube");
	    viewer.addCube(_cube[0], _cube[1], _cube[2], _cube[3], _cube[4], _cube[5],1,1,1,"cube");
	    }

	    if(!_poly1.empty() || !_poly2.empty()) {
	    viewer.removeShape("poly1");
	    viewer.removeShape("poly2");
	    pcl::PointCloud<pcl::PointXYZ>::Ptr p1 (_poly1.makeShared());
	    pcl::PointCloud<pcl::PointXYZ>::Ptr p2 (_poly2.makeShared());
	    //pcl::PointCloud<pcl::PointXYZ>::Ptr p(_poly);
	    viewer.addPolygon<pcl::PointXYZ>(p1,1.,1.,1.,"poly1");
	    viewer.addPolygon<pcl::PointXYZ>(p2,1.,1.,1.,"poly2");
	    }
	*/
	    viewer.removeAllPointClouds();
	    //viewer.updatePointCloud(*_clouds[0],_cloudNames[0]);
	    for (size_t i=0; i < _clouds.size(); i++) {
		viewer.addPointCloud(_clouds[i], hdlr[i],
				     Eigen::Vector4f (0.0f, 0.0f, 0.0f, 0.0f),
				     Eigen::Quaternionf (1.0f, 0.0f, 0.0f, 0.0f),
				     _cloudNames[i]);
		viewer.setPointCloudRenderingProperties(
			    pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,_cloudNames[i]);
	    }
	    _newCloud=false;
	}
	viewer.spinOnce(10,true);
	//range_image_widget->spinOnce (10,true);
	pcl_sleep(0.01);
    }

    BOOST_LOG_TRIVIAL(debug) << "Thread " << __FUNCTION__ << " stopping";
    viewer.removeAllPointClouds();
    viewer.spinOnce(100,true);
    viewer.close();

    _signal = 0;
    BOOST_LOG_TRIVIAL(debug) << "Thread " << __FUNCTION__ << " ends";
    return;
}

bool CloudViewPCL::filter(const Frame &in, Frame& out)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    toffy::Filter::setLoggingLvl();

    pcl::PCLPointCloud2Ptr planar;

    bool update=false;
    //cout << "cloudNames.size(): " << _cloudNames.size() << endl;
    //cout << "_clouds.size(): " << _clouds.size() << endl;
    for (size_t i=0;i<_cloudNames.size();i++) {
	if ( in.hasKey(_cloudNames[i])) {
	    //cout << "_cloudNames[i]: " << _cloudNames[i] << endl;
	    planar = boost::any_cast<pcl::PCLPointCloud2Ptr>(in.getData(_cloudNames[i]));
	    //cout << "planar: " << planar->header << endl;
	    //cout << "planar: " << planar->height << endl;
	    *_clouds[i] = *planar;
	    update = true;
	}
    }
    _newCloud = update;

    /* Boomerang
    if ( in.hasKey("cube")) {
	_cube = boost::any_cast<std::vector<float> >(in.getData("cube"));
	update=true;
    }

    if ( in.hasKey("poly1")) {
    _poly1 = boost::any_cast<pcl::PointCloud<pcl::PointXYZ> >(in.getData("poly1"));
	update=true;
    }

    if ( in.hasKey("poly2")) {
    _poly2 = boost::any_cast<pcl::PointCloud<pcl::PointXYZ> >(in.getData("poly2"));
    update=true;
    }
    */
    if (_signal == 0) {
	_signal = 1;
	_thread = boost::thread(boost::bind(&CloudViewPCL::loopViewer,this));
    }

    return true;
}

void CloudViewPCL::signal(int s) {
    _signal = s;
}
