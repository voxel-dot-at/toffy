#pragma once

#include <toffy/filter.hpp>

namespace toffy {
namespace detection {
class DetectedObject;

/**
 * @brief Blob segmentation - takes a b/w image and uses findcontours to detect
 *  individual regions
 * @ingroup Detection
 * @author VoXel Interaction Design <office@voxel.at>
 *
 * For detailed information see \ref blobs_page description page
 *
 */
class Blobs: public toffy::Filter {
public:
    Blobs();
    virtual ~Blobs();

    static const std::string id_name; ///< Filter identifier

    virtual bool filter(const toffy::Frame& in, toffy::Frame& out);

    //virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

private:
    std::string in_img, ///< Depth image
	in_ampl, ///< Amplitude image
	out_blobs; ///< List of detected blobs

    int _minSize; ///< Of the blob in # of pixels

    //bool amplFilter; ///< Flag to filter depth by amplitude
    //int minAmpl; ///< min amplitude, values bellow are set to 0

    int _morphoSize, ///< Size of the element
	_morphoIter, ///< Number of iteration
	_morphoType; ///< Type of operation. @see cv::MorphTypes

    bool refineBlobs, ///< Flag to activate the Fillflood refinement algorithm
	_morpho, ///< Flag(default:false) activate the morphology operation
	sharpenEdges, /// Flag @todo
	_filterInternals; ///< Flag(default:false) remove blobs with holes

    static std::size_t _filter_counter; ///< Internal Filter counter

    void findBlobs(cv::Mat& in, cv::Mat& ampl, int fc,
		   std::vector<toffy::detection::DetectedObject*>& list);

    void refineBlob(cv::Mat& dist, int fc,
		    toffy::detection::DetectedObject* o);
};
}
}
