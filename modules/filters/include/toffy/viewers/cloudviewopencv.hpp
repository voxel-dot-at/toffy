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

#include "toffy/filter.hpp"
//#include <opencv2/viz.hpp>

namespace toffy {

class CloudViewOpenCv : public Filter	{
    std::string _in_cloud;
    //cv::viz::Viz3d *sameWindow;
public:
    CloudViewOpenCv(): Filter("cloudviewopencv"), _in_cloud("cloud") {
	//sameWindow = new cv::viz::Viz3d();
    }
    virtual ~CloudViewOpenCv(){
	//delete sameWindow;
    }

    virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out) const;
};
}
