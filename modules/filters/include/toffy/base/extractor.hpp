#pragma once
/*
 * @file alignment.hpp
 *
 * Copyright (c) 2015
 * VoXel Interaction Design GmbH
 *
 * VoXel.at <office@voxel.at>
 * @author Angel Merino-Sastre <amerino@voxel.at>
 * @date 4 Sep 2014
 * @version 1.0-rc2
 *
 * @brief
 *
 * Description
 *
 *
 * Usage:
 */
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
