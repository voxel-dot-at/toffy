#pragma once

#include <vector>

#include "toffy/filter.hpp"
#include <boost/thread.hpp>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

namespace toffy {
/**
 * @brief 3d Viewer for PointCloud2 using PCL
 * @ingroup Viewers
 *
 * Show PointCloud2 (new PCL standard) int a viewer.
 * The viewer runs in a separated thread that updates continuoslly to provide
 * interaction.
 *
 * @todo There is a mayor issue with the viewer. In linux, GTK does not delete
 * the viewer in any case. When we try to instanciate a viewer again in the
 * same process, it just crash.
 *
 * \section ex1 Xml Configuration
 * @include cloudviewpcl.xml
 *
 */
class CloudViewPCL : public Filter {
public:

    static const std::string id_name; ///< Filter identifier
    CloudViewPCL();

    virtual ~CloudViewPCL();

    virtual bool filter(const Frame& in, Frame& out);

    virtual boost::property_tree::ptree getConfig() const;

    void updateConfig(const boost::property_tree::ptree &pt);

    /**
     * @brief Getter signal status
     * @param s
     */
    void signal(int s);

private:

    std::string _in_cloud;
    boost::thread _thread;
    volatile int _signal; ///< Interthread flag. Used for check the thread status
    double _coordSize;

    pcl::PCLPointCloud2Ptr _planar; ///< Aux

    std::vector<std::string> _cloudNames; /**< Keep all cloud names in Frame
	to be displayed*/
    std::vector<pcl::PCLPointCloud2Ptr> _clouds;
    volatile bool _newCloud; /*< Interthread flag to advice a runtime change in
	the clouds to display */

    static std::size_t _filter_counter;

    /**
     * @brief Internal loop called within the thread that keeps the viewer
     * running.
     */
    void loopViewer();

    //boomerang
    //std::vector<float> _cube;
    //pcl::PointCloud<pcl::PointXYZ> _poly1, _poly2;
    //pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;

};
}
