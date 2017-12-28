#pragma once

#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <toffy/filter.hpp>


namespace toffy {
namespace filters {
/**
 * @brief Distance/Amplitude correction via interpolation of an
 * amplitude-dependent correction term
 * @ingroup Filters
 *
 */
class DistAmpl: public toffy::Filter{
    // linear interpolation breakpoints and coefficients
    float *breaks,
	*coeffs;

    int blen,
	clen;


    std::string _in_ampl,
	_in_depth;

    std::vector< std::vector <double > > arr_coef,
	arr_log;
    std::vector< int > arr_breaks;

public:
    DistAmpl(std::string name="distAmpl");

    virtual bool filter(const toffy::Frame& in, toffy::Frame& out);


    virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

private:
    virtual bool f1(const toffy::Frame& in, toffy::Frame& /*out*/);

    float linearInterp(int measuredAmpl)
    {
	int i=1;
	while ( measuredAmpl < breaks[i] && i<blen ) {
	    i++;
	}
	// mA is now in breaks[i-1] .. breaks[i] interval

	// norm to [0..1[ interval:
	float mLocal = ( measuredAmpl - breaks[i-1] ) / ( breaks[i] - breaks[i-1]);

	// compute correction term as linear interpolation of the two factors:
	float res = mLocal * coeffs[i-1] + (1-mLocal) * coeffs[i];
	return res;
    }

    /**
     * @brief pchip lookup function
     * @param x
     * @return
     */
    double pchip(double measuredAmpl) {
	int blen = arr_breaks.size();

	double y,xlocal;
	int i=1;
	// find start:
	while ( measuredAmpl > arr_breaks[i] && i<blen) { // todo: log n lookup
	    i++;
	}
	if (i==blen) return measuredAmpl; // overflow - should not happen.

	double ampl0=arr_breaks[i-1];
	//double ampl1=arr_breaks[i];

	// matlab 'help mkpp':
	//COEFS(i,1)*(X-BREAKS(i))^(K-1) + COEFS(i,2)*(X-BREAKS(i))^(K-2) + ...
	//  COEFS(i,K-1)*(X-BREAKS(i)) + COEFS(i,K)

	xlocal = measuredAmpl-ampl0;
	std::vector<double> coefs = arr_coef[i-1];

	double x2 = xlocal*xlocal;
	double x3 = x2 * xlocal;
	y = coefs[0]*x3 + coefs[1]*x2 + coefs[2]*xlocal + coefs[3];

	return y;
    }

};

}}
