#include <iostream>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <boost/log/trivial.hpp>

#include "toffy/base/cond.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif

static std::string theName("cond");
static int counter=0;

Cond::Cond(): FilterBank(theName, counter++)
{
}


bool Cond::filter(const Frame &in, Frame& out) 
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    if (enabled) {
	return FilterBank::filter(in, out);
    }

    return true;
}

boost::property_tree::ptree Cond::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    return pt;
}
