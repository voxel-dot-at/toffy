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

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <boost/log/trivial.hpp>
#include <boost/any.hpp>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "toffy/filter_helpers.hpp"
#include "toffy/viewers/colorize.hpp"

using namespace toffy;
using namespace cv;
using namespace std;

std::size_t Colorize::filter_counter = 1;
const std::string Colorize::id_name = "colorize";

Colorize::Colorize()
    : Filter(Colorize::id_name, filter_counter),
      scale(1.f),
      max(1.5),
      min(0.01),
      in_img("depth"),
      out_img("colored"),
      colormap("jet")
{
    filter_counter++;
}

Colorize::~Colorize() {}

void Colorize::updateConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    scale = pt.get<double>("options.scale", scale);
    max = pt.get<double>("options.max", max);
    min = pt.get<double>("options.min", min);

    colormap = pt.get<string>("options.colormap", colormap);

    in_img = pt.get<string>("inputs.img", in_img);
    out_img = pt.get<string>("outputs.img", out_img);

    if (colormap == "jet") {
        colormap_value = COLORMAP_JET;
    } else if (colormap == "hot") {
        colormap_value = COLORMAP_HOT;
    } else if (colormap == "hsv") {
        colormap_value = COLORMAP_HSV;
    } else if (colormap == "rainbow") {
        colormap_value = COLORMAP_RAINBOW;
    } else {
        colormap_value = COLORMAP_JET;
    }
}

boost::property_tree::ptree Colorize::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.scale", scale);
    pt.put("options.max", max);
    pt.put("options.min", min);

    pt.put("options.colormap", colormap);

    pt.put("inputs.img", in_img);
    pt.put("outputs.img", out_img);

    return pt;
}

bool Colorize::filter(const Frame& in, Frame& out)
{
    matPtr img, colored;

    using namespace boost::posix_time;

    ptime start = microsec_clock::local_time();
    boost::posix_time::time_duration diff;

    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    ///// INPUT

    if (!in.hasKey(in_img)) {
        BOOST_LOG_TRIVIAL(warning) << "Could not find input " << in_img
                                   << ", filter  " << id() << " not applied.";
        return false;
    }
    img = in.getMatPtr(in_img);

    Mat& inp = *img;
    if (!inp.data) {
        BOOST_LOG_TRIVIAL(warning) << "missing image " << in_img << ", filter  "
                                   << id() << " not applied.";
        return false;
    }

    if (!out.hasKey(out_img)) {
        colored.reset(
            new Mat(Size(img->rows * scale, img->cols * scale), CV_8UC3));
    } else {
        colored = out.getMatPtr(out_img);
    }

    //// PROCESSING

    // convert input to grayscale
    // apply colormap
    // set invalid pixels to black
    // scale if needed
    // store result in outupt.

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "Colorize got: " << diff.total_microseconds();

    //show.convertTo(show,CV_8U,255.0/(1000-0),-255.0*0/(1000-0));
    if (max == min) {
        minMaxIdx(inp, &min, &max);
    }
    inp.convertTo(gray, CV_8U, 255.0 / (max - min), -255.0 * min / (max - min));

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug)
        << "Colorize gray2rgb: " << diff.total_microseconds();

    // color conversion...:
    applyColorMap(gray, col, colormap_value);

    bounds = (inp < min) | (inp > max) | (inp != inp);
    col.setTo(Scalar(0, 0, 0), bounds);

    if (scale != 1.f) {
        int method = scale < 1 ? INTER_AREA : INTER_LINEAR;
        resize(col, *colored, Size(), scale, scale, method);

        diff = boost::posix_time::microsec_clock::local_time() - start;
        BOOST_LOG_TRIVIAL(debug)
            << "Colorize resize: " << diff.total_microseconds();

    } else {
        *colored = col;
    }

    //imshow(name() + " " + out_img, col);

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "Colorize fin: " << diff.total_microseconds();

    //// OUTPUT
    out.addData(out_img, colored);

    return true;
}
