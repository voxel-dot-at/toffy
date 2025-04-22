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

#include <opencv2/imgproc.hpp>

#include "toffy/base/focus.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;





const std::string Focus::id_name = "focus";

Focus::Focus(): Filter(Focus::id_name),
		    in_img("ampl"), out_focus("focus")
{
}

void Focus::updateConfig(const boost::property_tree::ptree &pt) {
    LOGD << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    // boost::optional<const boost::property_tree::ptree& > ocvo = pt.get_child_optional( "options.cameraMatrix" );

    in_img = pt.get<string>("inputs.img",in_img);
    out_focus = pt.get<string>("outputs.focus",out_focus);
}

boost::property_tree::ptree Focus::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("inputs.img", in_img);
    pt.put("outputs.focus", out_focus);

    return pt;
}

bool Focus::filter(const Frame &in, Frame& out) {
	LOGD << __FUNCTION__ <<  " " << id();

	matPtr img;
	try {
		img = std::any_cast<matPtr >(in.getData(in_img));

	} catch(const std::bad_any_cast &) {
		LOGW <<
			"Could not cast input " << in_img <<
			", filter  " << id() <<" not applied.";
		return false;
	}

	{ 
	    //
	    // compute focus based on laplacian:
	    // cf. http://stackoverflow.com/questions/28717054/calculating-sharpness-of-an-image
	    //
	    Mat dst;
	    cv::Laplacian(*img, dst, CV_32F);
	    
	    cv::Scalar mu, sigma;
	    cv::meanStdDev(dst, mu, sigma);
	    
	    double focusMeasure = sigma.val[0] * sigma.val[0];


	    out.addData(out_focus, focusMeasure); 
	}

	return true;
}
