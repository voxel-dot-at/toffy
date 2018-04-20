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
#include <string>

#include "toffy/filter.hpp"

namespace cv {
class VideoWriter;
}

namespace toffy {

/** Obstacle detector based on local edge information
	 */
class VideoOut: public Filter
{
public:
    cv::Rect rect;
    bool debug; // show images or not

    VideoOut();
    virtual ~VideoOut();

    virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

    virtual void startSaving(const std::string& file, Frame& in);

    virtual void stopSaving();

    // settable parameters:

    double minDist; //< min distance
    double maxDist; //< max distance for scaling the output
    double minAmpl; //< min distance
    double maxAmpl; //< max distance for scaling the output

    double scale;   //< scale factor for distance and amplitude images

private:
    bool saving;
    cv::VideoWriter* writer;
    cv::Mat image;
};

}
