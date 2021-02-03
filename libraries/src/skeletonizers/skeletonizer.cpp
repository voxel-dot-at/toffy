/*
 * Skeletonizer.cpp
 *
 *  Created on: Dec 13, 2012
 *      Author: simon
 */

#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>

#include "toffy/skeletonizers/skeletonizer.hpp"

using namespace std;
using namespace cv;

Skeletonizer::Skeletonizer() {
	// TODO Auto-generated constructor stub

}

Skeletonizer::~Skeletonizer() 
{
	thickness.release();
	Input.release();
}
