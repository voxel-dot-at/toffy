#pragma once

#include <toffy/filter.hpp>
#include <toffy/detection/detectedObject.hpp>


/** @defgroup Tracking Tracking
 *
 * Tracking module
 *
 */

namespace toffy {
//! tracking namespace
namespace tracking {

/**
 * @brief distance where we consider an object changed position and it is not
 * a different object.
 *
 * @todo Move to config??
 */
const double maxMergeDistance = 20.;

/**
 * @brief Object tracker filter
 * @ingroup Tracking
 *
 * For detailed information see \ref tracker_page description page
 *
 */
class Tracker : public Filter
{
public:
    static const std::string id_name; ///< Filter identifier

    Tracker();
    virtual ~Tracker(){}

    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

    /**
     * @brief getBlobs
     * @return
     *
     * @todo Where is this been called
     */
    const std::vector<toffy::detection::DetectedObject* >& getDecObjs() const { return detObjs; }


private:
    std::string _in_vec, ///< Name of the next list of detected blobs
	_in_fc, ///< Camera frame counter
	_in_img; ///< Image from where the new blobs where detected
    std::string _out_img, ///< Name of the modified image
	_out_objects, ///< List of tracked objects
	_out_count; ///< # of detected objects
    double maxMergeDistance;

    static std::size_t _filter_counter; ///< Internal filter counter

    unsigned int _fc, _ts;
    bool _render_image;

    std::vector<toffy::detection::DetectedObject* > detObjs; ///<Internal list of tracked objects

    /**
     * @brief Debug views
     * @param depth
     *
     */
    void showObjects(cv::Mat& depth);
};

}
}
