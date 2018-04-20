#pragma once

#include <boost/property_tree/json_parser.hpp>

namespace toffy {

namespace Actions {

class Action
{

public:

    Action() {}
    ~Action() {}
    std::string id;

    void toJsonString(boost::property_tree::ptree& ptree) {
	ptree.add("action.id",id);
    }
    std::string toJsonString() {
	boost::property_tree::ptree jOut;
	std::stringstream ss;

	toJsonString(jOut);
	boost::property_tree::json_parser::write_json(ss,jOut,false);

	return ss.str();
    }

};

}
}
