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

#include <toffy/filter.hpp>

#include <toffy/cam/cameraParams.hpp>

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

    toffy::cam::cameraPtr cam;

    void findBlobs(cv::Mat& in, cv::Mat& ampl, int fc,
		   std::vector<toffy::detection::DetectedObject*>& list);

    void refineBlob(cv::Mat& dist, int fc,
		    toffy::detection::DetectedObject* o);
};
}
}
