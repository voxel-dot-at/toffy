#include <toffy/common/filenodehelper.hpp>
#include <sstream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/version.hpp>
#include <boost/foreach.hpp>
#include <boost/log/trivial.hpp>
#include <iostream>

using namespace toffy;
using namespace commons;

using namespace boost;
using namespace boost::property_tree;

using namespace std;

cv::FileStorage toffy::commons::loadOCVnode(
    const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    std::ostringstream oss;

    using namespace boost::property_tree::xml_parser;

    boost::property_tree::ptree oc;

    oc.add_child("opencv_storage", pt);

    xml_parser::write_xml(oss, oc);

    BOOST_LOG_TRIVIAL(debug) << oss.str();

    cv::FileStorage fs(oss.str(),
                       cv::FileStorage::MEMORY | cv::FileStorage::READ);

    return fs;
}

bool toffy::commons::checkOCVNone(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    boost::optional<string> attr = pt.get_optional<string>("<xmlattr>.type_id");
    if (attr.is_initialized() && *attr == "opencv-matrix") {
        return true;
    }
    return false;
}
