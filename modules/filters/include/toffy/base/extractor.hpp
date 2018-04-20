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

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif

#include "toffy/filter.hpp"

/**
 * @brief The Alignment class
 *
 * Class for finding needed transformation correction from
 * check markers.
 * The idea is to find the rotations that moves the marker
 * center positions to the real world coordinates positions.
 *
 * It is been done now only using local coordinates.
 * It tries to align the marker along the y , x and the z
 * coordinates.
 *
 * //TODO Load real coordinates from a file
 * //TODO Review calculation and write checks
 * //TODO write an exporter
 *
 */
namespace toffy {
 namespace filters {
	class Extractor : public Filter {

		cv::Rect roi;
		mutable std::vector<cv::Mat> values;

	 public:
		Extractor();
		Extractor(cv::Rect in): roi (in) {}
		virtual ~Extractor() {}

		int changeRoi(cv::Rect newRoi);

		int exportValuesCsv(std::string name);

		int exportMeanStdevCsv(std::string name);

		std::vector<cv::Mat> & getValues() {return values;}

		std::vector<cv::Mat> getRoiValues();

		cv::Rect & getRoi() {return roi;}

		virtual int loadConfig(const cv::FileNode &fn);

		virtual bool filter(const Frame& in, Frame& out) const;
	};
 }
}
