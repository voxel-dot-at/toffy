#include <boost/foreach.hpp>
#include <boost/log/trivial.hpp>

#include <toffy/common/filenodehelper.hpp>

#include "toffy/import/dataimporter.hpp"

using namespace toffy;
using namespace toffy::commons;
using namespace toffy::import;
using namespace cv;
using namespace std;

std::size_t DataImporter::_filter_counter = 1;

DataImporter::DataImporter(): Filter("dataImporter",_filter_counter) {
    _filter_counter++;
}

/*
int DataImporter::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const boost::property_tree::ptree& pt)";
    const boost::property_tree::ptree& dataImporter = pt.get_child(this->type());
    updateConfig(dataImporter);
    return 1;
}
*/

boost::property_tree::ptree DataImporter::getConfig() const {
    boost::property_tree::ptree pt;
    //boost::property_tree::ptree bta;
    //std::cout << "bta::getConfig "<<endl;

    pt = Filter::getConfig();
    //show all values
    for (size_t i = 0; i < _data.size(); i++) {
	//out.addData(_data.at(i).first, _data.at(i).second);
    }
    return pt;
}

void DataImporter::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);
    _data.clear();
    ptree pto = pt.get_child("options");
    //for(ptree::iterator v = pto.begin(); v != pto.end(); v++)
    //{
    BOOST_FOREACH(const ptree::value_type &v,
                  pt.get_child("options"))
    {
        if(checkOCVNone(v.second)) {
	    boost::property_tree::ptree os;
	    os.put_child(v.first,v.second);
	    cv::FileStorage fs = loadOCVnode(os);
	    Mat val;
	    fs.getFirstTopLevelNode() >> val;
	    //cout << fs.getFirstTopLevelNode().name() << endl;
	    _data.push_back(std::make_pair(fs.getFirstTopLevelNode().name(),val));
	    fs.release();
	} else
	    _data.push_back(std::make_pair(v.first,v.second));
    }
    return;
}

bool DataImporter::filter(const Frame& in,  Frame& out) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    for (size_t i = 0; i < _data.size(); i++) {
	out.addData(_data.at(i).first, _data.at(i).second);
    }
    return true;
}
