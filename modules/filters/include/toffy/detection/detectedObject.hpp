#pragma once

#include <toffy/toffy_export.h>
#include <toffy/toffy_config.h>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#endif


#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/circular_buffer.hpp>

#ifdef MSVC
  #define DLLExport __declspec( dllexport )
#else
  #define DLLExport /**/
#endif

/** @defgroup Detection Detection
 *
 * Detection module
 *
 */

namespace toffy {
//! Detection namespace
namespace detection {

/**
 * @brief Class to describe an object found in an image
 * @ingroup Detection
 *
 * This class unique identify every object and keep track of his features
 * like contour, center position, etc...
 *
 */
class /*DLLExport*/ TOFFY_EXPORT DetectedObject {
    static int COUNTER; ///< Internal object identifier
public:

    int id; ///< TODO

    boost::shared_ptr<boost::circular_buffer<DetectedObject* > > record;

    // object data changing per frame follows:

    std::vector<cv::Point> contour; ///< OpenCV contour of the object
    int idx; ///< Position in the list of contours
    boost::shared_ptr<std::vector<std::vector<cv::Point> > > contours; /**<
    List of all neighbor contours, usefull for look keep track of holes in object */
    boost::shared_ptr<std::vector<cv::Vec4i > > hierarchy; /**< List of connected objects */

    cv::Moments mo; ///< OpenCV moments
    cv::Point2f massCenter; ///< Mass center of the object calculated from the moments

    int fc; ///< Frame counter when the object was last detected
    int cts;  ///< Camera timestamp last detection
    boost::posix_time::ptime ts; ///< System timestamp last detection
    float size; ///< Size in pixels computed by contourArea()

    // persistent object data - frame invariant:
    cv::Scalar color; ///< Unique color for displaying
    int first_fc; ///< Frame counter when the object was first detected
    int first_cts; ///< Camera timestamp first detected
    boost::posix_time::ptime first_ts; ///< System timestamp first detection

    cv::Point2f firstCenter; // Where the object appeared

    /**
     * @brief DetectedObject
     */
    DetectedObject();

    /**
     * @brief Copy constructor
     */
    DetectedObject(const DetectedObject& object);

    /**
     * @brief ~DetectedObject
     */
    virtual ~DetectedObject();

    /**
     * @brief Update changes from blob detection to track current state.
     * used by the object tracker.
     * @param newDO
     *
     */
    void update(const DetectedObject& newDO);

    void init();

    DetectedObject * clone(const DetectedObject& object);

    DetectedObject& operator=( const DetectedObject& newDO );


    cv::Point3d massCenter3D;
    float massCenterZ;
};

}}
