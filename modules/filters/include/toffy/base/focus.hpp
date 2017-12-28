#pragma once

#include <toffy/filter.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif

namespace toffy {
namespace filters {
/**
 * @brief Compute a focus measure for the image sharpness
 * @ingroup Filters
 *
 * \section ex1 Xml Configuration
 * @include focus.xml
 *
 */
class Focus : public Filter {
public:
    static const std::string id_name; ///< Filter identifier

    Focus();
    virtual ~Focus() {}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

private:
    std::string in_img; ///< Name of the input image
    std::string out_focus; ///< Name of the key for the focus variable double  
};
}}
