#pragma once

#include <toffy/filter.hpp>

namespace toffy {
namespace detection {
/**
  *
@brief Square detector:
@ingroup Detection

For detailed information see \ref handDetect_page description page
 *
 */
class SquareDetect: public Filter
{
public:
    static const std::string id_name;
    SquareDetect();
    virtual ~SquareDetect();

    /** call the filter functions in the filter bank
     */
    bool filter(const Frame& in, Frame& out);

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

private:
    std::string in_depth, ///< depth input mat
	in_ampl, ///< ampl input mat
	in_blobs, ///< ampl detected object vector
	out_detect; ///<  detected objects visu
    cv::Mat _cameraMatrix;

    static std::size_t _filter_counter;
};

}}
