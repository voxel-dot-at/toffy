#pragma once
/*
 * @file bbox.hpp
 *
 * Copyright (c) 2015
 * VoXel Interaction Design GmbH
 *
 * VoXel.at <office@voxel.at>
 * @author 
 * @date 
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
	namespace f3d {
	    const int sampleModelUnknown=0;
	    const int sampleModelLine=1;
	    const int sampleModelPlane=2;

	    /** SampleConsensus analyses clouds with a selectable RANSAC model 
	     */
	    class SampleConsensus : public Filter {
	    public:
		SampleConsensus(): Filter("sampleConsensus"), 
				   in("cloud"),inliers("inliers"), 
				   outliers("outliers"), 
				   model("plane"), threshold(0.02), maxIters(1000) {}

		virtual ~SampleConsensus() {}

		virtual bool filter(const Frame& in, Frame& out);

		virtual int loadConfig(const boost::property_tree::ptree& pt);

		virtual boost::property_tree::ptree getConfig() const;

		void updateConfig(const boost::property_tree::ptree &pt);
	    private:
		std::string in, inliers, outliers, model;
		double threshold;
		int maxIters;
		int mdl; // the numeric value of the model

	    };
	}
    }
}

