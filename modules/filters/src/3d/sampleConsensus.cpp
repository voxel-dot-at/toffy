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

#include <pcl/range_image/range_image_planar.h>

#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>


#include <pcl/sample_consensus/sac.h>

#include <boost/date_time/posix_time/posix_time.hpp>


#include <boost/log/trivial.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "toffy/3d/sampleConsensus.hpp"

using namespace toffy;
using namespace toffy::filters::f3d;
using namespace cv;
using namespace std;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif


bool SampleConsensus::filter(const Frame &in, Frame& out) 
{
    using namespace boost::posix_time;
    
    ptime start = boost::posix_time::microsec_clock::local_time();
    ptime tick = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = tick - start;

    pcl::PCLPointCloud2Ptr planar;
    try {
        planar = boost::any_cast<pcl::PCLPointCloud2Ptr>(in.getData(this->in));
    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) <<
	    "Could not cast input "  <<
	    ", filter  " << id() <<" not applied.";
	return false;
    }
    /*
    pcl::SampleConsensusModelPtr scModel;
    if (model == "plane" ) {
	scModel = model_plane(new pcl::SampleConsensusModelPlane<pcl::PointWithRange>(planar));
    } else if (model == "line" ) {
	pcl::SampleConsensusModelLine<pcl::PointWithRange>::Ptr
	    model_line(new pcl::SampleConsensusModelLine<pcl::PointWithRange>(planar));
    } else {
	return false;
    }
    */

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::fromPCLPointCloud2(*planar,*cloud);

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
    
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane, threshold );

    ransac.setMaxIterations(maxIters);

    tick = boost::posix_time::microsec_clock::local_time();
    diff = tick - start;
    cout << id() << "::filter\t" << diff.total_milliseconds() << "\tpre sac" << endl;

    bool success = ransac.computeModel(1);
    if (!success) {
	cout << " could not ransac - skip!" << endl;
	return true;
    }

    tick = boost::posix_time::microsec_clock::local_time();
    diff = tick - start;
    cout << id() << "::filter\t" << diff.total_milliseconds() << "\tpost sac" << endl;

    std::vector<int> inliers;
    ransac.getInliers(inliers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pi(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr po(new pcl::PointCloud<pcl::PointXYZ>);

    if (inliers.size() ) {
	size_t idx=0;
	size_t pIdx=0;
	while ( pIdx < cloud->size() ) {
	    if ( pIdx == (unsigned int)inliers[idx]) {
		pi->push_back( cloud->at(pIdx));
		if (idx<inliers.size()) // or push a dummy value to inliers
		    idx++;
	    } else {
		po->push_back( cloud->at(pIdx));
	    }
	    pIdx++;
	}

	out.addData( this->inliers, pi);
	out.addData( this->outliers, po);
    }

    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);
    
    cout << "PLAN " << coeffs.transpose() << endl;
    /*
    float roll, pitch, yaw;	
    Eigen::Affine3f A;
    pcl::getEulerAngles ( coeffs, roll, pitch, yaw);

    cout << "PLAN " << coeffs.transpose() << " " << roll << " " << pitch << " " << yaw <<  endl;
    */


    tick = boost::posix_time::microsec_clock::local_time();
    diff = tick - start;
    cout << id() << "::filter\t" << diff.total_milliseconds() << "\tfin" << endl;

    return true;
}

/******************************************************************************************************
 */

int SampleConsensus::loadConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    updateConfig(pt.begin()->second );
    return 1;
}


boost::property_tree::ptree SampleConsensus::getConfig() const
{
    boost::property_tree::ptree pt;

    pt.put("inputs.cloud", in);

    pt.put("outputs.inliers", inliers);
    pt.put("outputs.outliers", outliers);

    pt.put("threshold", threshold);
    pt.put("maxIterations", maxIters);

    return pt;
}

void SampleConsensus::updateConfig(const boost::property_tree::ptree &pt) 
{
    using namespace boost::property_tree;

    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();
    
    in = pt.get("inputs.cloud", in);

    inliers = pt.get("outputs.inliers", inliers);
    outliers = pt.get("outputs.outliers", outliers);

    threshold = pt.get("threshold", threshold);
    maxIters = pt.get("maxIterations", maxIters);
}
