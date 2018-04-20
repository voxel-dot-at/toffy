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

  cv::FileStorage toffy::commons::loadOCVnode(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    std::ostringstream oss;

    using namespace boost::property_tree::xml_parser;

    boost::property_tree::ptree oc;
    //cout << pt.data() << endl;
    oc.add_child("opencv_storage", pt);
    /*#if (BOOST_VERSION > 105500 )
        boost::property_tree::xml_parser::xml_writer_settings<std::string> settings('\t', 1);
    #else
        boost::property_tree::xml_parser::xml_writer_settings<char> settings('\t', 1);
    #endif*/
    //boost::property_tree::xml_parser::write_xml(oss, oc, settings);
    xml_parser::write_xml(oss, oc);
    //cout << "oss.str(): " << oss.str() << endl;
    BOOST_LOG_TRIVIAL(debug) << oss.str();

    cv::FileStorage fs( oss.str() , cv::FileStorage::MEMORY | cv::FileStorage::READ );

    return fs;
  }


  bool toffy::commons::checkOCVNone(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    //cout << pt.data() << endl;
    boost::optional<string> attr = pt.get_optional<string>("<xmlattr>.type_id");
    if (attr.is_initialized() && *attr == "opencv-matrix") {
	return true;
    }
    return false;
  }
