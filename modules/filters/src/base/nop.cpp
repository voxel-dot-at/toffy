
#include <iostream>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <boost/log/trivial.hpp>

#include "toffy/base/nop.hpp"

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


Nop::Nop(): Filter("nop")
{
}

/*
int Nop::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const boost::property_tree::ptree& pt)";
    const boost::property_tree::ptree& nop = pt.get_child(this->type());

    loadGlobals(nop);

    updateConfig(nop);

    return true;
}

void Nop::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    setCommon(pt);

}
*/

bool Nop::filter(const Frame &in, Frame& out) 
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    out = in;

    return true;
}

boost::property_tree::ptree Nop::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    return pt;
}
