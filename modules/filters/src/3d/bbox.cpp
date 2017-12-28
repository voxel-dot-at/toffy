#include <string>
#include <iostream>
#include <fstream>
#include <limits.h>

#include <boost/log/trivial.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/common/transforms.h>

#include "toffy/3d/bbox.hpp"

using namespace toffy;
using namespace toffy::filters::f3d;
using namespace cv;
using namespace std;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif

template<typename P>
size_t bbox_filter(typename pcl::PointCloud<P>::Ptr in,
		   typename pcl::PointCloud<P>::Ptr inliers, // inside interval
		   typename pcl::PointCloud<P>::Ptr outliers, // outside interval
		   std::string axis, double min, double max)
{
	typename pcl::PassThrough<P> pass;
	cout << "filtering " << axis << "  " << min << " - " << max << endl;
	pass.setInputCloud (in);
	pass.setFilterFieldName (axis);
	pass.setFilterLimits (min, max);

	pass.setFilterLimitsNegative(false);
	pass.filter (*inliers);

	pass.setFilterLimitsNegative(true);
	pass.filter (*outliers);

	return inliers->size();
}

bool Split::filter(const Frame &in, Frame& out) 
{
    pcl::RangeImagePlanar::Ptr planar;
    try {
	planar = boost::any_cast<pcl::RangeImagePlanar::Ptr>(in.getData(_in_cloud));
    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) <<
	    "Could not cast input " << _in_cloud <<
	    ", filter  " << id() <<" not applied.";
	return false;
    }

    pcl::RangeImagePlanar::Ptr po(new pcl::RangeImagePlanar());
    pcl::RangeImagePlanar::Ptr pi(new pcl::RangeImagePlanar());

    bbox_filter<pcl::PointWithRange>(planar, pi, po, _axis, _min, _max);

    out.addData( _out_inliers, pi);
    out.addData(_out_outliers, po);
    return true;
}

/******************************************************************************************************
 */

int Split::loadConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();
    const boost::property_tree::ptree& split = pt.get_child("split");
    boost::optional<string> name = split.get_optional<string>("name");
    if (name.is_initialized()) {
	this->name(*name);
    }

    cout << "HAHA!!!" << id() << " " << pt.begin()->first << endl;
    updateConfig(pt.begin()->second );
    return 1;
}


boost::property_tree::ptree Split::getConfig() const
{
    boost::property_tree::ptree pt;

    pt.put("inputs.cloud", _in_cloud);

    pt.put("outputs.inliers", _out_inliers);
    pt.put("outputs.outliers", _out_outliers);

    pt.put("options.axis", _axis);
    pt.put("options.min", _min);
    pt.put("options.max", _max);

    return pt;
}

void Split::updateConfig(const boost::property_tree::ptree &pt) 
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();
    


    _in_cloud = pt.get("inputs.cloud", _in_cloud);

    _out_inliers = pt.get("outputs.inliers", _out_inliers);
    _out_outliers = pt.get("outputs.outliers", _out_outliers);

    _axis = pt.get("axis", _axis);
    _min = pt.get("min", _min);
    _max = pt.get("max", _max);

    _axis = pt.get("options.axis", _axis);
    _min = pt.get("options.min", _min);
    _max = pt.get("options.max", _max);

}
