#include <iostream>

#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include "toffy/smoothing/bilateral.hpp"

using namespace toffy;
using namespace toffy::filters::smoothing;
using namespace cv;
using namespace std;
namespace logging = boost::log;
namespace fs = boost::filesystem;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif

std::size_t Bilateral::_filter_counter = 1;
const std::string Bilateral::id_name = "backgroundSubs";

Bilateral::Bilateral() :Filter("bilateral"), 
			_in_img("depth"),
			_out_img("depth"),
			d(5), sigmaColor(10.), sigmaSpace(10.)
{}


bool Bilateral::filter(const Frame &in, Frame& out) 
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    boost::shared_ptr<cv::Mat> depth, ampl;
    boost::shared_ptr<cv::Mat> bi;

    try {
	depth = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_in_img));

    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) <<
	    "Could not cast input " << _in_img <<
	    ", filter  " << id() <<" not applied.";
	return false;
    }
    try {
	if (in.hasKey(_out_img))
	    bi = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_out_img));
	else
	    bi.reset(new Mat(depth->size(),depth->type()));
    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) <<
	    "Could not cast output " << _in_img <<
	    ", filter  " << id() <<" not applied.";
	return false;
    }

    BOOST_LOG_TRIVIAL(debug) << depth->type();
    BOOST_LOG_TRIVIAL(debug) << CV_32FC1;
    BOOST_LOG_TRIVIAL(debug) << (depth->type() == CV_32FC1);
    BOOST_LOG_TRIVIAL(debug) << (depth->data != depth->data);
    //BOOST_LOG_TRIVIAL(debug) << depth->size();
    Mat mask = *depth <= 0;
    Mat imgmask;
    depth->copyTo(imgmask,mask);
    std::cout << "hola" << std::endl;
    erode(imgmask,imgmask,Mat());
    std::cout << "hola" << std::endl;
    bilateralFilter(*depth, _dst, d, sigmaColor, sigmaSpace, BORDER_REPLICATE);
    std::cout << "hola" << std::endl;
    _dst = *depth + ((_dst-*depth) & imgmask);
    std::cout << "hola" << std::endl;
    //_dst /= mask;

    *bi = _dst.clone();

    
    out.addData(_out_img, bi);
    cout << id() << " wrote to " << _out_img << endl;

    return true;
}

/*
int Bilateral::loadConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();
    cout << "HAHA!!!" << id() << " " << pt.begin()->first << endl;
    updateConfig(pt.begin()->second );
    return 1;
}
*/

boost::property_tree::ptree Bilateral::getConfig() const
{
    boost::property_tree::ptree pt;
    pt = Filter::getConfig();
    pt.put("options.d", d);
    pt.put("options.sigmaColor", sigmaColor);
    pt.put("options.sigmaSpace", sigmaSpace);

    pt.put("inputs.img", _in_img);

    pt.put("outputs.img", _out_img);

    return pt;
}

void Bilateral::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    Filter::updateConfig(pt);
    d          = pt.get("options.d", d);
    sigmaSpace = pt.get("options.sigmaSpace", sigmaSpace);
    sigmaColor = pt.get("options.sigmaColor", sigmaColor);

    _in_img = pt.get("inputs.img", _in_img);

    _out_img = pt.get("outputs.img", _out_img);
}


