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
#include <boost/log/trivial.hpp>
#include <boost/any.hpp>

#include <opencv2/viz.hpp>

#include "toffy/filter_helpers.hpp"
#include "toffy/viewers/cloudviewopencv.hpp"

using namespace toffy;
using namespace cv;

CloudViewOpenCv::CloudViewOpenCv(): Filter("cloudviewopencv"), _in_cloud("cloud") {
	sameWindow = new cv::viz::Viz3d();
}

CloudViewOpenCv::~CloudViewOpenCv() { 
	delete sameWindow; 
}

int CloudViewOpenCv::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const boost::property_tree::ptree& pt)";
    const boost::property_tree::ptree& conf = pt.get_child(this->type());

    loadGlobals(conf);

    updateConfig(conf);

    return true;
}

void CloudViewOpenCv::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _in_cloud = pt.get<std::string>("inputs.cloud",_in_cloud);
}

boost::property_tree::ptree CloudViewOpenCv::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("inputs.cloud", _in_cloud);

    return pt;
}


bool CloudViewOpenCv::filter(const Frame &in, Frame& out) const {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

	matPtr img3d;
	try {
		img3d = boost::any_cast<matPtr>(in.getData(_in_cloud));
	} catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Could not cast input " << _in_cloud <<
			", filter  " << id() <<" not applied.";
		return false;
	}
	*sameWindow = viz::getWindowByName("Viz Demo");

	viz::WCloud cloud_widget(*img3d, viz::Color::green());
	sameWindow->showWidget("Coordinate Widget", viz::WCoordinateSystem());
	sameWindow->showWidget("Cloud", cloud_widget);

	sameWindow->spinOnce(30, true);
	return true;
}
