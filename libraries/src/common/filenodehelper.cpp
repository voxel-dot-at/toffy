#include <sstream>
#include <iostream>

#include <boost/property_tree/xml_parser.hpp>
// #include <boost/version.hpp>
// #include <boost/foreach.hpp>


#include <toffy/common/filenodehelper.hpp>

#include <toffy/logging.hpp>

using namespace toffy;
using namespace commons;

using namespace boost;
using namespace boost::property_tree;

using namespace std;

cv::FileStorage toffy::commons::loadOCVnode(
    const boost::property_tree::ptree& pt)
{
    LOGD << __FUNCTION__;
    std::ostringstream oss;

    using namespace boost::property_tree::xml_parser;

    boost::property_tree::ptree oc;

    oc.add_child("opencv_storage", pt);

    xml_parser::write_xml(oss, oc);

    LOGD << oss.str();

    cv::FileStorage fs(oss.str(),
                       cv::FileStorage::MEMORY | cv::FileStorage::READ);

    return fs;
}

bool toffy::commons::checkOCVNone(const boost::property_tree::ptree& pt)
{
    LOGD << __FUNCTION__;

    boost::optional<string> attr = pt.get_optional<string>("<xmlattr>.type_id");
    if (attr.is_initialized() && *attr == "opencv-matrix") {
        return true;
    }
    return false;
}
