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

#include "toffy/toffy_config.h"
#include "toffy/mux.hpp"

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#endif



/**
 * @brief
 *
 */
namespace toffy {
    namespace filters {
	namespace f3d {
	    class MuxMerge : public Mux {

	    public:
		MuxMerge(std::string name="muxMerge"): Mux(name), _out_cloud("merged") {}
		virtual ~MuxMerge() {}

		//virtual int loadConfig(const boost::property_tree::ptree& pt);
		virtual void updateConfig(const boost::property_tree::ptree &pt);

		virtual bool filter(const std::vector<Frame*>& in, Frame& out);
           private:
               /*
               // merge one input name into one output
               virtual bool mergeRangeImagePlanar(const std::vector<Frame*>& in, Frame& out, 
                                                  const std::string& inName, const std::string& outName);

               /// merge by iterating the cloud name array
               virtual bool mergeRangeImagePlanar(const std::vector<Frame*>& in, Frame& out, 
                                                  const std::string& outName);
               */
               std::vector<std::string> _clouds;
               std::string _out_cloud;

               std::vector<std::string> _cpy_fields;

	    };
	}
    }
}
