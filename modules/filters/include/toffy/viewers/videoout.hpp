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
