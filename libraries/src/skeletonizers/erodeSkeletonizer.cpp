#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>

#include "toffy/skeletonizers/erodeSkeletonizer.hpp"

using namespace std;
using namespace cv;
//using namespace rapidjson;

ErodeSkeletonizer::ErodeSkeletonizer()  {
    Debugging = false;
}

ErodeSkeletonizer::~ErodeSkeletonizer() {
}


void ErodeSkeletonizer::skeletonize(const cv::Mat& lines, cv::Mat& skel)
{
    this->skel(lines,skel);
}

Skeletonizer *ErodeSkeletonizer::Create()
{
    return new ErodeSkeletonizer();
}


void ErodeSkeletonizer::skel(const Mat& in, Mat& skel)
{
	// morphol. thinning, based on
	// http://felix.abecassis.me/2011/09/opencv-morphological-skeleton/
	Mat element = cv::getStructuringElement(MORPH_CROSS, Size(3, 3));

	Mat lines;

	lines = in;

	if (skel.rows != in.rows || skel.cols != in.cols || skel.type() != CV_8UC1) {
		skel.create(in.rows, in.cols, CV_8UC1);
	}
	cv::Mat temp(in.size(), CV_8UC1);
	skel.setTo(Scalar(0));

	Mat eroded;
	bool done;
	string nm="E_____3";
	int x=0;
	do {
		erode(lines, eroded, element);
		dilate(eroded, temp, element); // temp = open(img)

		subtract(lines, temp, temp);

		bitwise_or(skel, temp, skel);
		eroded.copyTo(lines);

		done = (cv::countNonZero(lines) == 0);
		//nm[2]='0'+x;
		//x++;
		//imshow(nm, lines);
	} while (!done);
}

/*
void ErodeSkeletonizer::configure(const rapidjson::Value &configObject)
{
    Debugging = configObject["Debug"].GetBool();
}
*/
