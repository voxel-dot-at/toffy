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


#include "toffy/filter.hpp"

/**
 * @brief
 *
 */
namespace toffy {
 namespace filters {
	class Range : public Filter {

		std::string _in_img, _out_img;
		double _min, _max;
		static std::size_t _filter_counter;
	 public:
		Range();
		virtual ~Range() {}


		//virtual int loadConfig(const boost::property_tree::ptree& pt);

		virtual boost::property_tree::ptree getConfig() const;
		void updateConfig(const boost::property_tree::ptree &pt);

		virtual bool filter(const Frame& in, Frame& out);
	};
 }
}
