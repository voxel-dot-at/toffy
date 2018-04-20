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
#include <pcl/common/transforms.h>

#include <boost/log/trivial.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "toffy/3d/transform.hpp"

using namespace toffy;
using namespace toffy::filters::f3d;
using namespace cv;
using namespace std;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif

std::size_t Transform::_filter_counter = 1;
const std::string Transform::id_name = "transform";

Transform::Transform(): Filter(Transform::id_name,_filter_counter),
    _in_cloud("cloud"), _out_cloud(_in_cloud)
{
    _filter_counter++;
}

/*
int Transform::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    const boost::property_tree::ptree& tree = pt.get_child(this->type());

    loadGlobals(tree);

    updateConfig(tree);

    return true;
}
*/

bool Transform::filter(const Frame &in, Frame& out) const {
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
	Eigen::Affine3f tr;
	tr.setIdentity();
	for (size_t i = 0; i < _actions.size(); i++) {
		if (_operations[i] == Transform::rotation) {
			Eigen::Matrix3f m;
			m = Eigen::AngleAxisf(_actions[i][0], Eigen::Vector3f::UnitX())
			  * Eigen::AngleAxisf(_actions[i][1],  Eigen::Vector3f::UnitY())
			  * Eigen::AngleAxisf(_actions[i][2], Eigen::Vector3f::UnitZ());
			tr = m * tr;
		} else if (_operations[i] == Transform::translation) {
			Eigen::Translation3f t(_actions[i][0],
					_actions[i][1],_actions[i][2]);
			tr = t *tr;
		} else if (_operations[i] == Transform::scaling) {
			//Eigen::Affine3f m = ;
			tr = Eigen::Scaling((float)_actions[i][0],(float)_actions[i][1],(float)_actions[i][2]) *tr;
		} else
			BOOST_LOG_TRIVIAL(warning) <<
				"Operation unknown.";
	}

	pcl::transformPointCloud(*planar, *planar, tr);
	out.addData(_out_cloud, planar);
	return true;
}

boost::property_tree::ptree Transform::getConfig() const
{
    boost::property_tree::ptree pt, actions;
    pt = Filter::getConfig();

    for (size_t i=0; i < _actions.size();i++) {
	boost::property_tree::ptree action;
	if (_operations[i] == Transform::rotation) {
	    action.put("rotation",_actions[i]);
	} else if (_operations[i] == Transform::translation) {
	    action.put("translation",_actions[i]);

	} else if (_operations[i] == Transform::scaling) {
	    action.put("scaling",_actions[i]);
	}
	// quirk to create array elements:
	actions.push_back(std::make_pair("", action));
    }

    pt.add_child("actions", actions);

    pt.put("inputs.cloud", _in_cloud);

    pt.put("outputs.cloud", _out_cloud);

    return pt;
}

void Transform::updateConfig(const boost::property_tree::ptree &pt) 
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _actions.clear();
    _operations.clear();
    try {
	BOOST_FOREACH(const ptree::value_type &v, pt.get_child("actions")) {
	    BOOST_FOREACH(const ptree::value_type &w, v.second) {

		std::vector<double> vec;
		BOOST_LOG_TRIVIAL(debug) << "w.first " << w.first;
		BOOST_LOG_TRIVIAL(debug) << w.second.data();
		std::stringstream ss(w.second.data());
		boost::property_tree::ptree jvec;
		boost::property_tree::json_parser::read_json(ss,jvec);
		BOOST_LOG_TRIVIAL(debug) << "jvec.size() " << jvec.size();
		BOOST_FOREACH(const ptree::value_type &x, jvec) {
		    BOOST_LOG_TRIVIAL(debug) << "x.second.data()" << x.second.data();
		    vec.push_back(x.second.get<double>(""));
		}
		BOOST_LOG_TRIVIAL(debug) << "vec.size()" << vec.size();
		if (vec.size() == 3) {
		    _actions.push_back(cv::Vec3d(vec[0],vec[1],vec[2]));
		    if(w.first == "rotation")
			_operations.push_back(Transform::rotation);
		    else if(w.first == "translation")
			_operations.push_back(Transform::translation);
		    else if(w.first == "scaling")
			_operations.push_back(Transform::scaling);
		}
	    }
	}
    } catch (const std::exception& ex) {
	BOOST_LOG_TRIVIAL(debug) << "Transform values are wrong. Ex:" << ex.what();
    }

    //_minAmpl = pt.get("options.minAmpl", _minAmpl);
    //_maxAmpl =  pt.get("options.maxAmpl", _maxAmpl);

    _in_cloud = pt.get("inputs.cloud", _in_cloud);

    _out_cloud = pt.get("outputs.cloud", _out_cloud);


}

