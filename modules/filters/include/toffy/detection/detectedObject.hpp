/*
   Copyright 2018 Simon Vogl <svogl@voxel.at>
                  Angel Merino-Sastre <amerino@voxel.at>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#pragma once

#include <opencv2/imgproc.hpp>


#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/circular_buffer.hpp>

#include "toffy/toffy_export.h"


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

    int id; ///< object id, set by blob detection or tracking algorithms

    std::shared_ptr<boost::circular_buffer<DetectedObject* > > record;

    // object data changing per frame follows:

    std::vector<cv::Point> contour; ///< OpenCV contour of the object
    int idx; ///< Position in the list of contours
    std::shared_ptr<std::vector<std::vector<cv::Point> > > contours; /**<
    List of all neighbor contours, usefull for look keep track of holes in object */
    std::shared_ptr<std::vector<cv::Vec4i > > hierarchy; /**< List of connected objects */

    cv::Moments mo; ///< OpenCV moments
    cv::Point2f massCenter; ///< Mass center of the object calculated from the moments
    double logHu[7]; ///< OpenCV hu moments, log scaled

    int fc; ///< Frame counter when the object was last detected
    int cts;  ///< Camera timestamp last detection
    boost::posix_time::ptime ts; ///< System timestamp last detection
    float size; ///< Size in pixels computed by contourArea()
    cv::Rect bbox; ///< bounding box of the object

    // persistent object data - frame invariant:
    cv::Scalar color; ///< Unique color for displaying
    unsigned int first_fc; ///< Frame counter when the object was first detected
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

typedef std::shared_ptr<DetectedObject> DetObjPtr;
typedef std::vector<DetectedObject*> DetectedObjects;

typedef std::shared_ptr<DetectedObjects> DetObjectsPtr;

}
}
