#include <boost/log/trivial.hpp>
#include <boost/any.hpp>

#include "toffy/viewers/cloudviewopencv.hpp"

using namespace toffy;
using namespace cv;

int CloudViewOpenCv::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const boost::property_tree::ptree& pt)";
    const boost::property_tree::ptree& rectify = pt.get_child(this->type());

    loadGlobals(rectify);

    updateConfig(rectify);

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

/*
int CloudViewOpenCv::loadConfig(const FileNode &fn) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

	FileNode cloudview = fn;
	if (!checkFileNode(cloudview))
		return 0;

	//cloudview["scale"] >> _scale;
	//cloudview["max_width"] >> _max_width;

	//Look for inputs
	FileNode ios = cloudview["inputs"];
	if (ios.empty()) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Missing inputs for filter " <<	id() <<
			" ... using defauts: " <<
			"\tcloud: " << _in_cloud;
	} else {
		ios["cloud"] >> _in_cloud;
	}

	std::string name;
	cloudview["name"] >> name;
	if (!name.empty())
		this->name(name);

	return 1;
}
*/

bool CloudViewOpenCv::filter(const Frame &in, Frame& out) const {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

	boost::shared_ptr<cv::Mat> img3d;
	try {
		img3d = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_in_cloud));
	} catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Could not cast input " << _in_cloud <<
			", filter  " << id() <<" not applied.";
		return false;
	}
	//*sameWindow = viz::getWindowByName("Viz Demo");

	//viz::WCloud cloud_widget(*img3d, viz::Color::green());
	//sameWindow->showWidget("Coordinate Widget", viz::WCoordinateSystem());
	//sameWindow->showWidget("Cloud", cloud_widget);

	//sameWindow->spinOnce(30, true);
	return true;
}

