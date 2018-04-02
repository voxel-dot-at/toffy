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
#include <string>
#include <iostream>
#include <fstream>
#include <limits.h>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "toffy/base/extractor.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;
namespace fs = boost::filesystem;

#ifdef CM_DEBUG
const bool dbg=true;
#else
const bool dbg=false;
#endif


Extractor::Extractor(): Filter("extractor") {
}

/**
 *
 * compute std deviation and eliminate outliers
 *
 */
template<typename T> double stdDev(const vector<T>& list, double avg, double minimum) {
	double s=0.;
	int skipped=0;
	for (size_t i=0;i< list.size() ; i++) {
		if ( isnan(list[i]) ||
		     list[i] < minimum) {
			skipped++;
			continue;
		}
		s +=  ( list[i] - avg ) * ( list[i] - avg );
	}
	cerr << "skips " << skipped << endl;
	return sqrt( s / (list.size() - skipped));
}

int Extractor::loadConfig(const FileNode &fn) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
	/*fs::path file(configFile);

	if ( !fs::exists(file) && fs::is_regular_file(file) ) {
		BOOST_LOG_TRIVIAL(error) << "File " << configFile << " does not exist or invalid.";
		return -1;
	}
	if ( boost::algorithm::to_lower_copy(file.extension().string()) != string(".xml") ) {
		BOOST_LOG_TRIVIAL(error) << "Wrong file extension: " << file.extension().string()
			<< " != .xml";
		return -1;
	}

	FileStorage storage(configFile, FileStorage::READ);
	if(!storage.isOpened()) {
		BOOST_LOG_TRIVIAL(error) << "Failed to open file " << configFile;
		return -1;
	}*/

	FileNode roisZone = fn;

	if (fn.name() != "extractor")
		roisZone = fn["extractor"];

	roisZone["x"] >> roi.x;
	roisZone["y"] >> roi.y;
	roisZone["width"] >> roi.width;
	roisZone["height"] >> roi.height;
	string name;
	roisZone["name"] >> name;
	if (!name.empty())
		this->name(name);

	return 1;
}

bool Extractor::filter(const Frame &in, Frame& out) const {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();
	out = in;

	if (in.hasKey("depth")) {
		BOOST_LOG_TRIVIAL(debug) << "depth found";
		boost::shared_ptr<cv::Mat> d = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData("depth"));
		Mat aux = d->clone();
		values.push_back(aux);
		return true;
	} else
		BOOST_LOG_TRIVIAL(info) << "NO depth";
	return false;

//int Extractor::accumulate(cv::Mat &img) {

	//values.push_back(img.clone());
}

int Extractor::changeRoi(cv::Rect newRoi) {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
	roi = newRoi;
	return 0;
}

int Extractor::exportValuesCsv(std::string name) {
#if OCV_VERSION_MAJOR >= 3
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
	std::vector<cv::Mat> roiVals = getRoiValues();
	std::ofstream outfile (name.c_str());
	for (size_t i=0; i < roiVals.size(); i++) {
		outfile << cv::format(roiVals[i],Formatter::FMT_CSV) << endl;
	}
	outfile.close();
	return 1;
#else
#warning Extractor: csv export is supported with OpenCV 3+ only!! 
	return 0;
#endif
}

int Extractor::exportMeanStdevCsv(std::string name) {
#if OCV_VERSION_MAJOR >= 3
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
	if (values.size() == 0) {
		BOOST_LOG_TRIVIAL(warning) << "extractor values empty.";
		return 0;
	}
	Mat mask = cv::Mat::zeros(values[0].size(), CV_8U); // all 0
	mask(roi) = 1;
	vector<Mat> valids;
	Mat validsCnt = cv::Mat::zeros(roi.size(), CV_32F);
	Mat meanValues = cv::Mat::zeros(roi.size(), CV_32F);
    //BOOST_LOG_TRIVIAL(debug) << roi;
	for (size_t i=0; i < values.size(); i++) {
		Mat vlds = values[i](roi) > 0.0;
		vlds &= std::numeric_limits<float>::quiet_NaN() != values[i](roi);
		valids.push_back(vlds);
		Mat vlsd_32;
		vlds.convertTo(vlsd_32,CV_32F,1/255.);
		validsCnt += vlsd_32;
		//BOOST_LOG_TRIVIAL(info) << validsCnt;
		Mat vals = values[i](roi);
		vals.convertTo(vals,CV_32F);
		vals.copyTo(vals,vlds);
		meanValues += vals;
	}
    //BOOST_LOG_TRIVIAL(debug) << values[0].size();
	//BOOST_LOG_TRIVIAL(info) << validsCnt;
	meanValues /= validsCnt;
	//Scalar meanValues = mean(values, mask);
	//BOOST_LOG_TRIVIAL(info) << meanValues;

	Mat stdValues = cv::Mat::zeros(roi.size(), meanValues.type());
	//std::vector<cv::Mat> std;
	for (size_t i=0; i < values.size(); i++) {
		Mat aux = values[i](roi);
		aux.convertTo(aux,CV_32F);
		aux.copyTo(aux,valids[i]);
		Mat local_means;
		meanValues.copyTo(local_means,valids[i]);
		aux -=  local_means;
		pow (aux, 2.0, aux);
		stdValues += aux;
	}
	stdValues /= validsCnt;
	sqrt( stdValues, stdValues );

	std::ofstream outfile (string(name + ".csv").c_str());

	outfile << "avg;" << endl;
	outfile << cv::format(meanValues, cv::Formatter::FMT_CSV) << std::endl;
	outfile << std::endl << "stdev;" << endl;
	outfile << cv::format(stdValues, cv::Formatter::FMT_CSV) << std::endl;
	outfile << std::endl << "valids out of : " << values.size() << ";" << endl;
	outfile << cv::format(validsCnt, cv::Formatter::FMT_CSV) << std::endl;

	outfile.close();
	return 1;
#else
#warning Extractor: csv export is supported with OpenCV 3+ only!! 
	return 0;
#endif
}

std::vector<cv::Mat> Extractor::getRoiValues() {
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
	std::vector<cv::Mat> output;
	for (size_t i=0; i < values.size(); i++) {
		Mat aux(values[i](roi));
		if (aux.data)
			output.push_back(aux);
		else
			BOOST_LOG_TRIVIAL(debug) << "Roi " << roi <<
				"does not below to image " << i+1 <<
				" of size " << values[i].size();
	}
	return output;
}
