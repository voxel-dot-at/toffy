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

#include <boost/filesystem.hpp>

#include "toffy/filter_helpers.hpp"
#include <boost/algorithm/string/case_conv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
# include <opencv2/highgui/highgui.hpp>

#include "toffy/smoothing/bilateral.hpp"

using namespace toffy;
using namespace toffy::filters::smoothing;
using namespace cv;
using namespace std;
namespace logging = boost::log;
namespace fs = boost::filesystem;


std::size_t Bilateral::_filter_counter = 1;

const std::string Bilateral::id_name = "bilateral";

Bilateral::Bilateral() :Filter(Bilateral::id_name), 
			_in_img("depth"),
			_out_img("depth"),
			d(5), sigmaColor(10.), sigmaSpace(10.)
{}


bool Bilateral::filter(const Frame &in, Frame& out) 
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    matPtr depth, ampl;
    matPtr bi;

    try {
	depth = boost::any_cast<matPtr >(in.getData(_in_img));

    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) <<
	    "Could not cast input " << _in_img <<
	    ", filter  " << id() <<" not applied.";
	return false;
    }
    try {
	if (in.hasKey(_out_img))
	    bi = boost::any_cast<matPtr >(in.getData(_out_img));
	else
	    bi.reset(new Mat(depth->size(),depth->type()));
    } catch(const boost::bad_any_cast &) {
	BOOST_LOG_TRIVIAL(warning) <<
	    "Could not cast output " << _in_img <<
	    ", filter  " << id() <<" not applied.";
	return false;
    }

    BOOST_LOG_TRIVIAL(debug) << depth->type();
    BOOST_LOG_TRIVIAL(debug) << CV_32FC1;
    BOOST_LOG_TRIVIAL(debug) << (depth->type() == CV_32FC1);
    BOOST_LOG_TRIVIAL(debug) << (depth->data != depth->data);
    //BOOST_LOG_TRIVIAL(debug) << depth->size();
    Mat mask = *depth <= 0;
    Mat imgmask;
    depth->copyTo(imgmask,mask);
    std::cout << "hola" << std::endl;
    erode(imgmask,imgmask,Mat());
    std::cout << "hola" << std::endl;
    bilateralFilter(*depth, _dst, d, sigmaColor, sigmaSpace, BORDER_REPLICATE);
    std::cout << "hola" << std::endl;
    _dst = *depth + ((_dst-*depth) & imgmask);
    std::cout << "hola" << std::endl;
    //_dst /= mask;

    *bi = _dst.clone();

    
    out.addData(_out_img, bi);
    cout << id() << " wrote to " << _out_img << endl;

    return true;
}

/*
int Bilateral::loadConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();
    cout << "HAHA!!!" << id() << " " << pt.begin()->first << endl;
    updateConfig(pt.begin()->second );
    return 1;
}
*/

boost::property_tree::ptree Bilateral::getConfig() const
{
    boost::property_tree::ptree pt;
    pt = Filter::getConfig();
    pt.put("options.d", d);
    pt.put("options.sigmaColor", sigmaColor);
    pt.put("options.sigmaSpace", sigmaSpace);

    pt.put("inputs.img", _in_img);

    pt.put("outputs.img", _out_img);

    return pt;
}

void Bilateral::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    Filter::updateConfig(pt);
    d          = pt.get("options.d", d);
    sigmaSpace = pt.get("options.sigmaSpace", sigmaSpace);
    sigmaColor = pt.get("options.sigmaColor", sigmaColor);

    _in_img = pt.get("inputs.img", _in_img);

    _out_img = pt.get("outputs.img", _out_img);
}
