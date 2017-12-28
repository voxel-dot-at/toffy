#pragma once

#include <string>

#include <toffy/filter.hpp>

namespace toffy {
    namespace filters {

	/**
	 * @brief The OffsetCorr class performs distance offset correction depending on modulation frequency.
	 */
	class OffsetCorr: public Filter{
	public:
	    OffsetCorr();

	    virtual bool filter(const toffy::Frame& in, toffy::Frame& out);

	    //virtual int loadConfig(const boost::property_tree::ptree& pt);
	    virtual boost::property_tree::ptree getConfig() const;
	    virtual void updateConfig(const boost::property_tree::ptree &pt);

	private:
	    std::vector<double> offsets;

	    std::string in_ampl, in_depth;
	};
    }
}

