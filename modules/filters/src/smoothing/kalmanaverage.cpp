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
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "toffy/filter_helpers.hpp"
#include "toffy/smoothing/kalmanaverage.hpp"

#include <iostream>
#include <fstream>

#include <math.h>

using namespace std;
using namespace cv;
using namespace toffy::filters::smoothing;

std::size_t toffy::filters::smoothing::KalmanAverage::_filter_counter = 1;
const std::string toffy::filters::smoothing::KalmanAverage::id_name =
    "kalmanAverage";

KalmanAverage::KalmanAverage()
    : Filter(KalmanAverage::id_name),
      _in_img("depth"),
      out_img("depth"),
      processNoiseCov(1e-5),
      measurementNoiseCov(1e-1),
      skipZeros(true)
{
}

KalmanAverage::~KalmanAverage() {}

/*
int KalmanAverage::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "(const
boost::property_tree::ptree& pt)"; const boost::property_tree::ptree& tree =
pt.get_child(this->type());

    loadGlobals(tree);

    updateConfig(tree);

    return true;
}
*/

void KalmanAverage::updateConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _in_img = pt.get<string>("inputs.img", _in_img);
    out_img = pt.get<string>("outputs.img", out_img);

    processNoiseCov = pt.get<float>("options.processNoiseCov", processNoiseCov);
    measurementNoiseCov =
        pt.get<float>("options.measurementNoiseCov", measurementNoiseCov);
    skipZeros = pt.get<bool>("options.skipZeros", skipZeros);
}

boost::property_tree::ptree KalmanAverage::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("inputs.img", _in_img);
    pt.put("outputs.img", out_img);

    pt.put("options.processNoiseCov", processNoiseCov);
    pt.put("options.measurementNoiseCov", measurementNoiseCov);
    pt.put("options.skipZeros", skipZeros);

    return pt;
}

bool KalmanAverage::filter(const toffy::Frame& in, toffy::Frame& /*out*/)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();
    boost::shared_ptr<cv::Mat> img;

    try {
        img = boost::any_cast<boost::shared_ptr<cv::Mat> >(in.getData(_in_img));
    } catch (const boost::bad_any_cast&) {
        BOOST_LOG_TRIVIAL(warning) << "Could not cast input " << _in_img
                                   << ", filter  " << id() << " not applied.";
        return false;
    }
    float* imgptr;
    int vKFpos = 0;
    if (!vKF) {
        vKF.reset(new std::vector<cv::KalmanFilter>(img->size().area()));
        for (int row = 0; row < img->rows; ++row) {
            imgptr = img->ptr<float>(row);

            for (int col = 0; col < img->cols; ++col) {
                cv::KalmanFilter& kf = vKF->at(vKFpos);
                // vKFpos = ( ( (row+1)+((img->rows)*row) ) * (col+1) ) -1;
                kf.init(2, 1, 0);
                kf.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);

                setIdentity(kf.measurementMatrix);
                setIdentity(kf.processNoiseCov, Scalar::all(1e-5));
                setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
                setIdentity(kf.errorCovPost, Scalar::all(1));

                kf.statePost.at<float>(0) = imgptr[col];
                vKFpos++;
                // cout << "vKFpos: " << vKFpos << endl;
                // cout << "imgptr[col]: " << imgptr[col] << endl;
                // cout << "kf.statePost.at<float>(0): " <<
                // kf.statePost.at<float>(0) << endl;
            }
        }
    } else {
        Mat measurement = Mat::zeros(1, 1, CV_32F);
        Mat estimated;
        for (int row = 0; row < img->rows; ++row) {
            imgptr = img->ptr<float>(row);

            for (int col = 0; col < img->cols; ++col) {
                cv::KalmanFilter& kf = vKF->at(vKFpos);

                if (std::numeric_limits<float>::quiet_NaN() == imgptr[col] ||
                    (skipZeros && imgptr[col] <= 0.0)) {
                    imgptr[col] = imgptr[col];  // nothing to estimate..
                    vKFpos++;

                    // kf.statePost.at<float>(0) =imgptr[col] ;
                    continue;
                }
                // last state was nan..:
                if (std::numeric_limits<float>::quiet_NaN() ==
                    kf.statePost.at<float>(0)) {
                    // re-init state
                    setIdentity(kf.measurementMatrix);
                    setIdentity(kf.processNoiseCov, Scalar::all(1e-5));
                    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
                    setIdentity(kf.errorCovPost, Scalar::all(1));

                    kf.statePost.at<float>(0) = imgptr[col];
                }

                // vKFpos = ( ( (row+1)+((img->rows)*row) ) * (col+1) ) -1;
                kf.predict();

                // measurement = Mat::zeros(3, 1, CV_32F);
                measurement.at<float>(0) = imgptr[col];

                estimated = kf.correct(measurement);
                imgptr[col] = estimated.at<float>(0);
                vKFpos++;
            }
        }
    }

    return true;
}
