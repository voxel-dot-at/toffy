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
#include <string>
#include <iostream>
#include <fstream>
#include <limits.h>


#include <boost/foreach.hpp>
#include <boost/log/trivial.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include "toffy/3d/muxMerge.hpp"

using namespace toffy;
using namespace toffy::filters::f3d;
using namespace cv;
using namespace std;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif

/*
int MuxMerge::loadConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << id();
    const boost::property_tree::ptree& node = pt.get_child("muxMerge");

    updateConfig(node);


    return 1;
}
*/

void MuxMerge::updateConfig(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);


    try {
	BOOST_FOREACH(const ptree::value_type &v, pt.get_child("inputs")) {
	    _clouds.push_back(v.second.data());
	}
    } catch (const std::exception& ex) {
	BOOST_LOG_TRIVIAL(debug) << "Wrong inputs. Ex:" << ex.what();
    }

    _out_cloud = pt.get<std::string>("outputs.cloud",_out_cloud);

    update = true;

}

bool MuxMerge::filter(const std::vector<Frame*>& in, Frame& out) 
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    pcl::PCLPointCloud2Ptr output;

    try {
	output = boost::any_cast<pcl::PCLPointCloud2Ptr>(in[0]->getData(_out_cloud));
    } catch(const boost::bad_any_cast &) {
	output.reset(new pcl::PCLPointCloud2());
	out.addData(_out_cloud, output );
	BOOST_LOG_TRIVIAL(debug) <<
	    "Could not cast output " << _out_cloud <<
	    ", filter  " << id() <<". Created.";
    }

    pcl::PCLPointCloud2Ptr first;

    try {
	first = boost::any_cast<pcl::PCLPointCloud2Ptr>(in[0]->getData(_clouds[0]));
    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) <<
	    "Could not cast input " << _clouds.front() <<
	    ", filter  " << id() <<" not applied.";
	return false;
    }
    pcl::copyPointCloud(*first, *output);
    //pcl::concatenatePointCloud(first,first,output)
    //pcl::PointCloud<pcl::PointXYZ> outputXYZ;
    //pcl::fromPCLPointCloud2 (*first, outputXYZ);

    for (size_t i = 1; i < in.size(); i++) {
	pcl::PCLPointCloud2Ptr rest;
	try {
	    rest = boost::any_cast<pcl::PCLPointCloud2Ptr>(in[i]->getData(_clouds[i]));
	} catch(const boost::bad_any_cast &) {
	    BOOST_LOG_TRIVIAL(warning) <<
		"Could not cast input " << _clouds[i] << " i: " << i <<
		", filter  " << id() <<" not applied.";
	    return false;
	}

	pcl::concatenate(*output, *rest, *output);
	//pcl::PointCloud<pcl::PointXYZ> restXYZ;
	//pcl::fromPCLPointCloud2 (*rest, restXYZ);
	//outputXYZ += restXYZ;
    }

    //pcl::toPCLPointCloud2(outputXYZ, *output);

    return true;
}
