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

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

//#include <sencam/BtaWrapper.hpp>
#include <toffy/filter.hpp>
#include <toffy/base/extractor.hpp>

#include <toffy/base/distAmpl.hpp>

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;

DistAmpl::DistAmpl(std::string name): Filter(name), _in_ampl("ampl"), _in_depth("depth")
{
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "DA";
}


bool DistAmpl::f1(const toffy::Frame& in, toffy::Frame& /*out*/)
{
	matPtr ampl = in.getMatPtr(_in_ampl);
	matPtr d = in.getMatPtr(_in_depth);

	for (int y=0;y<ampl->rows; y++) {
	    float* rowD = d->ptr<float>(y);
	    short* rowA = ampl->ptr<short>(y);

	    for (int x=0;x<ampl->cols;x++) {
		short a = *rowA;

		if (a > 500 && a < 12000 ) { // todo:limit to valid distances?

		    float corr = linearInterp(a);

		    *rowD *= corr; // apply ampl. correction
		}

		rowA++; rowD++;
	    }
	}

	return true;
}


bool DistAmpl::filter(const toffy::Frame& in, toffy::Frame& out)
{

	f1(in, out);

	return true;
}

int DistAmpl::loadConfig(const boost::property_tree::ptree& pt)
{
    const boost::property_tree::ptree& fltr = pt.get_child(this->name());
    updateConfig(fltr);
    return 1;
}

void DistAmpl::updateConfig(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    //_minAmpl = pt.get<double>("options.minAmpl",_minAmpl);
    //_maxAmpl = pt.get<double>("options.maxAmpl",_maxAmpl);

    _in_depth = pt.get<string>("inputs.depth",_in_depth);
    _in_ampl = pt.get<string>("inputs.ampl",_in_ampl);

    blen=clen=6;
    coeffs = new float[blen];
    breaks = new float[blen];
    breaks[0]=  300;    coeffs[0]=1. / 0.9842; //Added manually
    breaks[1]=  500;    coeffs[0]=1. / 0.9982;
    breaks[2]= 1000;    coeffs[1]=1. / 1.0083;
    breaks[3]= 2000;    coeffs[2]=1. / 1.0407;
    breaks[4]= 5500;    coeffs[3]=1. / 1.0656;
    breaks[5]=15000;    coeffs[4]=1. / 1.0800;

    cout << "DA CONF!!" << _in_ampl << endl;

    //_out_depth = pt.get<string>("outputs.depth",_out_depth);
    //_out_ampl = pt.get<string>("outputs.ampl",_out_ampl);


    //update = true;

}

boost::property_tree::ptree DistAmpl::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    //pt.put("options.timeStamped", _tsd);
    return pt;
}

/*
int DistAmpl::loadConfig(const cv::FileNode &fn)
{
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
	cout << "DA CONF START!!" << in_ampl <<endl;

	FileNode amplRange = fn;
	if (!checkFileNode(amplRange)) {
		cout << "DA ??? " << fn.name() << " " << id() << endl;
		return 0;
	}
	//Look for inputs
	FileNode ios = fn["inputs"];
	if (ios.empty()) {
		BOOST_LOG_TRIVIAL(warning) <<
			"Missing inputs for filter " <<	id() <<
			" ... using defaults: in_a/d: " << in_ampl << " " << in_depth;
	} else {
		ios["ampl"] >> in_ampl;
		ios["depth"] >> in_depth;
		BOOST_LOG_TRIVIAL(debug) << id() << " input ampl: " << in_ampl << " d " << in_depth;
	}


	blen=clen=6;
	coeffs = new float[blen];
	breaks = new float[blen];
	breaks[0]=  300;    coeffs[0]=1. / 0.9842; //Added manually
	breaks[1]=  500;    coeffs[0]=1. / 0.9982;
	breaks[2]= 1000;    coeffs[1]=1. / 1.0083;
	breaks[3]= 2000;    coeffs[2]=1. / 1.0407;
	breaks[4]= 5500;    coeffs[3]=1. / 1.0656;
	breaks[5]=15000;    coeffs[4]=1. / 1.0800;

	cout << "DA CONF!!" << in_ampl << endl;
	return 1;
}
*/
