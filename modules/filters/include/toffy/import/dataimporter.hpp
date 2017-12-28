#pragma once

#include "toffy/filter.hpp"

namespace toffy {
namespace import {

    class DataImporter : public Filter
    {
	static std::size_t _filter_counter;
	std::vector<std::pair<std::string,boost::any> > _data;
    public:
	DataImporter();
	virtual ~DataImporter(){}

	//virtual int loadConfig(const boost::property_tree::ptree& pt);

	virtual boost::property_tree::ptree getConfig() const;
	void updateConfig(const boost::property_tree::ptree &pt);

	virtual bool filter(const Frame& in, Frame& out);

    };

}
}
