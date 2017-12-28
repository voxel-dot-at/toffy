#include <iostream>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "toffy/base/amplitudeRange.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;
namespace fs = boost::filesystem;

#ifndef CM_DEBUG
    const bool dbg=false;
#endif

std::size_t AmplitudeRange::_filter_counter = 1;
const std::string AmplitudeRange::id_name = "amplitudeRange";

AmplitudeRange::AmplitudeRange(): Filter(AmplitudeRange::id_name, _filter_counter),
  _in_ampl("ampl"), _in_depth("depth"), _out_ampl(_in_ampl),
  _out_depth(_in_depth), _minAmpl(0), _maxAmpl(25000)
{
    _filter_counter++;
}

void AmplitudeRange::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _minAmpl = pt.get<double>("options.minAmpl",_minAmpl);
    _maxAmpl = pt.get<double>("options.maxAmpl",_maxAmpl);

    _in_depth = pt.get<string>("inputs.depth",_in_depth);
    _in_ampl = pt.get<string>("inputs.ampl",_in_ampl);

    _out_depth = pt.get<string>("outputs.depth",_out_depth);
    _out_ampl = pt.get<string>("outputs.ampl",_out_ampl);
}

boost::property_tree::ptree AmplitudeRange::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.minAmpl", _minAmpl);
    pt.put("options.maxAmpl", _maxAmpl);

    pt.put("inputs.depth", _in_depth);
    pt.put("inputs.ampl", _in_ampl);

    pt.put("outputs.depth", _out_depth);
    pt.put("outputs.ampl", _out_ampl);

    return pt;
}

bool AmplitudeRange::filter(const Frame &in, Frame& out) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

	boost::shared_ptr<cv::Mat> ampl, depth;
	try {
		ampl = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_in_ampl));
	} catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Could not cast input " << _in_ampl <<
			", filter  " << id() <<" not applied.";
		return false;
	}
	
	try {
		depth = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_in_depth));
	} catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Could not cast input " << _in_depth <<
			", filter  " << id() <<" not applied.";
		return false;
	}

	//TODO we need new data at the output
	//TODO handle aux vars creation
	Mat minMask = *ampl > _minAmpl;
	Mat maxMask = *ampl < _maxAmpl;
	Mat mask = minMask & maxMask;
	//mask != mask;
	Mat newAmpl, newDepth;
	ampl->copyTo(newAmpl, mask);
	depth->copyTo(newDepth, mask);

	newAmpl.copyTo(*ampl);
	newDepth.copyTo(*depth);

	out.addData(_out_ampl,ampl);
	out.addData(_out_depth,depth);

	return true;
}
