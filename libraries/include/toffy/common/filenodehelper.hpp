#pragma once

#include <toffy/toffy_export.h>
#include <toffy/toffy_config.h>
#include <boost/property_tree/ptree.hpp>

#if OCV_VERSION_MAJOR >= 3
    #  include <opencv2/core.hpp>
#else
    #  include <opencv2/core/core.hpp>
#endif

#if defined(MSVC)
  #define DLLExport __declspec( dllexport )
#else
  #define DLLExport /**/
#endif

namespace toffy {
namespace commons {

    /*DLLExport*/ TOFFY_EXPORT cv::FileStorage loadOCVnode(const boost::property_tree::ptree& pt);
    /*DLLExport*/ TOFFY_EXPORT bool checkOCVNone(const boost::property_tree::ptree& pt);

}
}
