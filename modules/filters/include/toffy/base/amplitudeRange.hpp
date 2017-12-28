#pragma once

#include "toffy/filter.hpp"

namespace toffy {
namespace filters {

/**
 * \brief Filters amplitude data and it corresponding pixels in the depth image
 * \ingroup Filters
 *
 * For detailled information see \ref amplitudeRange_page description page
 *
 */
class AmplitudeRange : public Filter {
public:
    static const std::string id_name; ///< Filter identifier

    AmplitudeRange();
    virtual ~AmplitudeRange() {}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);
private:
    std::string _in_ampl,
	_in_depth,
	_out_ampl,
	_out_depth;
    double _minAmpl,
	_maxAmpl;
    static std::size_t _filter_counter;
};
}
}
