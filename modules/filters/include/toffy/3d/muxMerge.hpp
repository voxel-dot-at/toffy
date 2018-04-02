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
