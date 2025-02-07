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

// windows wants
#define _USE_MATH_DEFINES
#include <cmath>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "toffy/base/polar2cart.hpp"
#include "toffy/common/filenodehelper.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;
namespace logging = boost::log;
namespace fs = boost::filesystem;

std::size_t Polar2Cart::_filter_counter = 1;
const std::string Polar2Cart::id_name = "polar2cart";

Polar2Cart::Polar2Cart()
    : Filter(Polar2Cart::id_name, _filter_counter),
      _fovx(-1),
      _fovy(-1),
      _in_img("img"),
      _out_img(_in_img)
{
    _filter_counter++;
}

void Polar2Cart::updateConfig(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _fovx = pt.get<double>("options.fovx", _fovx);
    _fovy = pt.get<double>("options.fovy", _fovy);

    boost::optional<const boost::property_tree::ptree &> ocvo =
        pt.get_child_optional("options.cameraMatrix");
    if (ocvo.is_initialized()) {
        if (toffy::commons::checkOCVNone(*ocvo)) {
            boost::property_tree::ptree os;
            os.put_child("cameraMatrix", *ocvo);
            cv::FileStorage fs = commons::loadOCVnode(os);
            cout << fs.getFirstTopLevelNode().name() << endl;
            fs.getFirstTopLevelNode() >> _cameraMatrix;
            fs.release();
        } else
            BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
        BOOST_LOG_TRIVIAL(debug) << "Node cameraMatrix is not opencv.";
    } else
        BOOST_LOG_TRIVIAL(debug) << "Node options.cameraMatrix not found.";

    _in_img = pt.get<string>("inputs.img", _in_img);
    _in_fovx = pt.get<string>("inputs.fovx", _in_fovx);
    _in_fovy = pt.get<string>("inputs.fovy", _in_fovy);

    _out_img = pt.get<string>("outputs.img", _out_img);
}

boost::property_tree::ptree Polar2Cart::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();
    pt.put("options.x_ang", _fovx);
    pt.put("options.y_ang", _fovy);

    pt.put("inputs.img", _in_img);
    pt.put("inputs.fovx", _in_fovx);
    pt.put("inputs.fovy", _in_fovy);

    pt.put("outputs.img", _out_img);

    return pt;
}

bool Polar2Cart::filter(const Frame &in, Frame & /*out*/)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    matPtr img;
    try {
        img = boost::any_cast<matPtr >(in.getData(_in_img));
    } catch (const boost::bad_any_cast &) {
        BOOST_LOG_TRIVIAL(error) << "Could not cast input " << _in_img
                                 << ", filter  " << id() << " not applied.";
        return false;
    }

    if (_cameraMatrix.data) {
        double noV, apertureWidth = (45 / 1000) * img->size().width,
                    apertureHeight = (45 / 1000) * img->size().height;
        Point2d np;
        calibrationMatrixValues(_cameraMatrix, img->size(), apertureWidth,
                                apertureHeight, _fovx, _fovy, noV, np, noV);
    }

    if (!_in_fovx.empty()) {
        try {
            _fovx = boost::any_cast<double>(in.getData(_in_fovx));
        } catch (const boost::bad_any_cast &) {
            BOOST_LOG_TRIVIAL(warning) << "Could not read input " << _in_fovx;
        }
    }
    if (!_in_fovy.empty()) {
        try {
            _fovy = boost::any_cast<double>(in.getData(_in_fovy));
        } catch (const boost::bad_any_cast &) {
            BOOST_LOG_TRIVIAL(warning) << "Could not read input " << _in_fovy;
        }
    }

    if (_fovx <= 0. || _fovy <= 0.) {
        BOOST_LOG_TRIVIAL(warning)
            << "No FoV data, filter " << id() << " not applied.";
        return false;
    }

    // TODO we need new data at the output
    // TODO handle aux vars creation
    Mat new_img = *img;

    float coef_h = (M_PI * _fovx / 180) / new_img.cols,
          coef_v = (M_PI * _fovy / 180) / new_img.rows;

    for (int i = 0; i < new_img.rows; i++) {
        for (int j = 0; j < new_img.cols; j++) {
            if (new_img.at<float>(i, j) < 65. && new_img.at<float>(i, j) > 0. &&
                new_img.at<float>(i, j) == new_img.at<float>(i, j)) {
                // TODO check type and channels
                new_img.at<float>(i, j) *=
                    (cos(coef_h * (abs(j - ((new_img.cols / 2) - 1))))) *
                    (cos(coef_v * (abs(i - ((new_img.rows / 2) - 1)))));
            }
        }
    }

    return true;
}
