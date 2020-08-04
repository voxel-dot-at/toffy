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
#include <errno.h>

#include <iostream>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/highgui.hpp>
#  include <opencv2/imgproc.hpp>
#  include <opencv2/imgproc/types_c.h>
#  include <opencv2/videoio.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#  include <opencv2/video/video.hpp>
#endif

#include <boost/log/trivial.hpp>

#include "toffy/viewers/videoout.hpp"

using namespace toffy;
using namespace cv;
using namespace std;

const int ofs=20; // lines ; before that we have text...

VideoOut::VideoOut() : scale(2), saving(false), writer(0)
{
}

VideoOut::~VideoOut()
{
	delete writer;
	writer = 0;
}

int VideoOut::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const boost::property_tree::ptree& pt)";
    const boost::property_tree::ptree& rectify = pt.get_child(this->type());

    loadGlobals(rectify);

    updateConfig(rectify);

    return true;
}

void VideoOut::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    minDist = pt.get<double>("options.min_dist" ,minDist);
    maxDist = pt.get<double>("options.max_dist",maxDist);
    minAmpl = pt.get<double>("options.min_ampl",minAmpl);
    maxAmpl = pt.get<double>("options.max_ampl",maxAmpl);
    scale = pt.get<double>("options.scale",scale);

    //_in_cloud = pt.get<std::string>("inputs.cloud",_in_cloud);
}

boost::property_tree::ptree VideoOut::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.min_dist", minDist);
    pt.put("options.max_dist", maxDist);
    pt.put("options.min_ampl", minAmpl);
    pt.put("options.max_ampl", maxAmpl);
    pt.put("options.scale", scale);

    return pt;
}


bool VideoOut::filter(const Frame& in, Frame& /*out*/) 
{
	if (writer && writer->isOpened()) {	
		matPtr depthP = boost::any_cast< matPtr >(in.getData("depth"));
		matPtr amplP = boost::any_cast< matPtr >(in.getData("amplitudes"));

		Mat amp, dis; // todo: keep 

		image = Scalar(0);

		amplP->convertTo(amp,CV_8U,255.0/(maxAmpl-minAmpl),-255.0*minAmpl/(maxAmpl-minAmpl));
		depthP->convertTo(dis,CV_8U,255.0/(maxDist-minDist),-255.0*minDist/(maxDist-minDist));

		if (scale != 1.0) {
			resize(amp,amp,Size(0,0),scale,scale);
			resize(dis,dis,Size(0,0),scale,scale);
		}

		cvtColor(amp, amp, COLOR_GRAY2BGR);
		cvtColor(dis, dis, COLOR_GRAY2BGR);

		Mat d(image, Rect(       0 , ofs , dis.cols , dis.rows));
		Mat a(image, Rect(dis.cols , ofs , amp.cols , amp.rows));

		amp.copyTo(a);
		dis.copyTo(d);

		stringstream t;
		//t << "F:" << in.fc() << " depth " << setprecision(3)<< minDist << "-" << setprecision(3)<<maxDist;
		putText(image,t.str(),Point(5,20)
			, FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));

		t.str("");
		t << "ampl " << (int)minAmpl << "-" << (int)maxAmpl;
		putText(image,t.str(),Point(5+dis.cols,20)
			, FONT_HERSHEY_PLAIN, 1, Scalar(255,255,128));

		imshow("image", image);
		*writer << image;
	}
	return true;
}

void VideoOut::startSaving(const std::string& file, Frame& in)
  {
  if (writer)
    delete writer;
  writer = new VideoWriter();

  // init image:
  matPtr dis = boost::any_cast< matPtr >(in.getData("depth"));
  matPtr amp = boost::any_cast< matPtr >(in.getData("amplitudes"));


  image = Mat(amp->rows*scale + ofs, (amp->cols+dis->cols)*scale, CV_8UC3);
#if OCV_VERSION_MAJOR >= 3
  bool success = writer->open(file, writer->fourcc('X','2','6','4'), 20, image.size());
#else
  bool success = writer->open(file, CV_FOURCC('X','2','6','4'), 20, image.size());
#endif
  if (!success) {
    cerr << "Opening video file " << file << " failed! " << strerror(errno) << endl;
  }
  saving = success;
}

void VideoOut::stopSaving()
{
	delete writer;
	writer = 0;
  saving = false;
}
