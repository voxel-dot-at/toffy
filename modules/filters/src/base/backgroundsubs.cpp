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
#include <iostream>
#include <fstream>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <toffy/smoothing/average.hpp>

#include <toffy/base/backgroundsubs.hpp>


using namespace std;
using namespace cv;
using namespace toffy;
using namespace filters;

std::size_t toffy::filters::BackgroundSubs::_filter_counter = 1;
const std::string toffy::filters::BackgroundSubs::id_name = "backgroundSubs";

BackgroundSubs::BackgroundSubs():
    Filter(id_name,_filter_counter), in_img("depth"), out_img("depth"),
    _in_mask("test.d"), _creation(true),_filterSpotNoise(false),
    _median(true), _neighbours(3), _ite(20), _offset(0.05)
{
    _filter_counter++;
}

BackgroundSubs::~BackgroundSubs() {}

void BackgroundSubs::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    in_img = pt.get<string>("inputs.img",in_img);
    _in_mask = pt.get<string>("inputs.mask",_in_mask);

    _creation = pt.get<bool>("options.creation",_creation);
    _ite = pt.get<int>("options.frames",_ite);
    _offset = pt.get<double>("options.offset",_offset);

    _filterSpotNoise = pt.get<bool>(
		"options.filterSpotNoise", _filterSpotNoise);
    _median = pt.get<bool>("options.medianFilter", _median);

    if (avgdImg) { // adjust distance map:
	loadAvgData(avgdImg);
    }
}

boost::property_tree::ptree BackgroundSubs::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("inputs.img", in_img);
    pt.put("inputs.mask", _in_mask);

    pt.put("options.creation", _creation);
    pt.put("options.frames", _ite);
    pt.put("options.offset", _offset);
    pt.put("options.filterSpotNoise", _filterSpotNoise);
    pt.put("options.medianFilter", _median);

    return pt;
}

bool BackgroundSubs::filter(const toffy::Frame& in, toffy::Frame& out)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();
    matPtr inImg;
    //matPtr outFg, outBg;
    //int fc = in.getInt(btaFc);

    try {
	inImg = in.getMatPtr(in_img);
    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) << "Could not cast input " <<
				      in_img << ", filter  " <<
				      id() <<" not applied.";
	return false;
    }


    if (true) { // nan points to zero distance
	Mat tmp = *inImg;
	Mat nan = tmp != tmp;
	//imshow("NAN", nan);
	Mat zero(nan.size(), inImg->type());
	zero = Scalar(0);
	zero.copyTo(tmp,nan);
    }

    if (_median)
	medianBlur(*inImg, *inImg, 3);

    if (_creation) {
	if (!_avg) {
	    //cout << id() << " START AVERAGING! " << endl;
	    _avg.reset(new toffy::filters::smoothing::Average());
	    boost::property_tree::ptree pt;
	    pt.put("options.size", _ite);
	    pt.put("inputs.img", in_img);
	    pt.put("outputs.img", "avgd_img");
	    _avg->updateConfig(pt);
	}

	_avg->filter(in, out);
	//std::cout << "SIZE: " << static_cast<toffy::filters::smoothing::Average*>(_avg.get())->size() << std::endl;
	if (static_cast<toffy::filters::smoothing::Average*>(_avg.get())->size() >= static_cast<unsigned int>(_ite)) {

	    try {
		avgdImg = out.getMatPtr("avgd_img");
	    } catch(const boost::bad_any_cast &) {
		BOOST_LOG_TRIVIAL(warning)
			<< "Could not cast input avgd_img, filter " <<
			   id() <<" not applied.";
		return false;
	    }

	    ofstream f;
	    f.open (_in_mask.c_str(), ios::out | ios::binary );
	    f.write ( (char*)avgdImg->data, avgdImg->size().area()*sizeof(float));
	    f.close();


	    _creation = false; // make system restartable
	    _avg.reset();

	    // update depth mask to match limits
	    loadAvgData(avgdImg);

	    cout << id() << " DONE AVERAGING! " << endl;
	}
    }

    if (!avgdImg) {

	bool success = loadAvgData(inImg);

	if (!success)
	    return true;
    }

    cout << "Creating mask " << endl;

    Mat mask/* = *inImg > *avgdImg;*/;
    compare(*inImg, *avgdImg, mask, CMP_LT);
    //cv::imshow("mask1", mask);
    mask|= *avgdImg == 0;
    //cv::imshow("mask", mask);
    Mat tmp;
    inImg->copyTo(tmp,mask);
    //cv::imshow("done", tmp);
    //waitKey(0);

    if (_filterSpotNoise) {
	// spot noise: remove pixels with have not enough neighbours.

	Mat s = tmp>0; // truth value -> 255
	s -= 254;      // clamp to 0..1 and count neighbourhood (includes the pixel itself!):
	
	Mat out;
	boxFilter(s,out,-1,Size(3,3), Point(-1,-1), false);
	// copy over everything with minimal connectivity:
	inImg.reset();
	tmp.copyTo(*inImg, (out > (1+_neighbours)) );

    } else {
	tmp.copyTo(*inImg);
    }
    //out.addData(in_img, avgdImg);

    return true;
}

bool BackgroundSubs::loadAvgData(matPtr inImg)
{
    cout << id() << " Loading image " << endl;
    BOOST_LOG_TRIVIAL(debug) << "Loading image ";
    ifstream f(_in_mask.c_str(), ios::in | ios::binary );
    if (f) {
	f.seekg (0, f.end);
	int length = f.tellg();
	f.seekg (0, f.beg);

	char *buffer = new char [length];
	f.read (buffer,length);
	f.close();
	avgdImg.reset(new Mat(inImg->size(),inImg->type(),buffer));
	avgdImg->copyTo(*avgdImg);
	//delete buffer;
	//cv::imshow("loaded", *avgdImg);

	//cout << "subs image " << endl;
	Mat rem;
	add(*avgdImg, _offset, rem, *avgdImg > 0);

	//*avgdImg = (*avgdImg > 0) -0.05;
	//cout << "subs done image " << rem << endl;
	rem .copyTo(*avgdImg);
	//cv::imshow("offset", rem);
	//waitKey(30);
    } else {
	cout << id() << ": no depth data yet - skipping"<<endl;
	return false;
    }
    return true;
}
