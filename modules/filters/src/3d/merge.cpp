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

#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/transforms.h>


#include "toffy/3d/merge.hpp"

using namespace toffy;
using namespace toffy::filters::f3d;
using namespace cv;
using namespace std;

#ifdef CM_DEBUG
static const bool dbg=true;
#else
static const bool dbg=false;
#endif

std::size_t Merge::_filter_counter = 1;
const std::string Merge::id_name = "merge";

Merge::Merge(): Filter(Merge::id_name,_filter_counter), _out_cloud("merged")
{
    _filter_counter++;
}

int Merge::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    //const boost::property_tree::ptree& tree = pt.get_child(this->type());

    Filter::loadConfig(pt);

    if (_clouds.size() < 2) {
	    BOOST_LOG_TRIVIAL(warning) <<
		    "Missing input clouds for filter " << id();
	    _clouds.clear();
	    return -1;
    }

    return true;
}

void Merge::updateConfig(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    try {
	BOOST_FOREACH(const ptree::value_type &v, pt.get_child("inputs")) {
	   _clouds.push_back(v.second.data());
	}
    } catch (const std::exception& ex) {
	BOOST_LOG_TRIVIAL(debug) << "Input values are wrong. Ex:" << ex.what();
    }

    _out_cloud = pt.get<string>("outputs.cloud", _out_cloud);


}

boost::property_tree::ptree Merge::getConfig() const
{
    boost::property_tree::ptree pt;
    pt = Filter::getConfig();

    for (size_t i=0; i < _clouds.size();i++) {
	pt.put("inputs.cloud", _clouds[i]);
    }

    pt.put("outputs.cloud", _out_cloud);

    return pt;
}

/*int Merge::loadConfig(const FileNode &fn) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

	Filter::loadConfig(fn);
	FileNode merge = fn;

	//Look for inputs
	FileNode ios = merge["inputs"];
	for(FileNodeIterator it = ios.begin() ; it != ios.end(); ++it) {
		string input = (string)*it;
		BOOST_LOG_TRIVIAL(debug) << input;
		_clouds.push_back(input);
	}
	if (_clouds.size() < 2) {
		Bboost::property_tree::ptree Transform::getConfig() const
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
}OOST_LOG_TRIVIAL(warning) <<
			"Missing input clouds for filter " <<	id();
		return -1;
	}

	ios = merge["outputs"];
	if (ios.empty()) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Missing outputs for filter " <<	id() <<
			" ... using defauts: " << endl <<
			"cloud: " << _out_cloud;
	} else {
		ios["cloud"] >> _out_cloud;
		BOOST_LOG_TRIVIAL(debug) << "output cloud: " << _out_cloud;
	}

	return 1;
}*/

bool Merge::filter(const Frame &in, Frame& out) const {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

	pcl::RangeImagePlanar::Ptr planar;
	try {
	    planar = boost::any_cast<pcl::RangeImagePlanar::Ptr>(in.getData(_clouds.front()));
	} catch(const boost::bad_any_cast &) {
	    BOOST_LOG_TRIVIAL(warning) <<
		    "Could not cast input " << _clouds.front() <<
		    ", filter  " << id() <<" not applied.";
	    return false;
	}

	for (size_t i = 1; i < _clouds.size(); i++) {
		pcl::RangeImagePlanar::Ptr planar2;
		try {
		    planar2 = boost::any_cast<pcl::RangeImagePlanar::Ptr>(in.getData(_clouds[i]));
		} catch(const boost::bad_any_cast &) {
		    BOOST_LOG_TRIVIAL(warning) <<
			    "Could not cast input " << _clouds[i] <<
			    ", filter  " << id() <<" not applied.";
		    return false;
		}
		*planar += *planar2;
	}


	out.addData(_out_cloud, planar);
	return true;
}

