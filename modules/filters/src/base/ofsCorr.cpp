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

#include <boost/log/trivial.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/imgproc.hpp>
#else
#  include <opencv2/imgproc/imgproc.hpp>
#endif


#include <toffy/base/ofsCorr.hpp>

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;

OffsetCorr::OffsetCorr(): Filter("offsetCorr"), in_ampl("ampl"), in_depth("depth")
{
	BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << "OFS";
}


bool OffsetCorr::filter(const toffy::Frame& in, toffy::Frame& out)
{
	matPtr ampl = in.getMatPtr(in_ampl);
	matPtr d = in.getMatPtr(in_depth);

	int mf = in.getUInt("mf");

	float fi = mf/5000000.f;
	int idx = mf/5000000;
	if (fi == 1.5) { // 7.5mhz
	    idx=1;
	} else if ( fi != idx || idx>6 ) {
	   BOOST_LOG_TRIVIAL(error)  << "ERROR CONVERTING TO IDX! " << (mf/5000000.) ;
	}

	//cout << "OFSCOR idx " << idx << " " << offsets[idx] <<  endl;
	*d += offsets[idx] ;

	return true;
}

/*
int OffsetCorr::loadConfig(const boost::property_tree::ptree& pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    const boost::property_tree::ptree& tree = pt.get_child(this->type());

    loadGlobals(tree);

    updateConfig(tree);

    return true;
}
*/

void OffsetCorr::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    in_ampl = pt.get<string>("inputs.ampl", in_ampl);
    in_depth = pt.get<string>("inputs.depth", in_depth);

    if (!offsets.size())
	offsets.resize(7, 0.0);

    offsets[0] = pt.get<float>("options.ofs_0", offsets[0]);
    offsets[1] = pt.get<float>("options.ofs_1", offsets[1]);
    offsets[2] = pt.get<float>("options.ofs_2", offsets[2]);
    offsets[3] = pt.get<float>("options.ofs_3", offsets[3]);
    offsets[4] = pt.get<float>("options.ofs_4", offsets[4]);
    offsets[5] = pt.get<float>("options.ofs_5", offsets[5]);
    offsets[6] = pt.get<float>("options.ofs_6", offsets[6]);
}

boost::property_tree::ptree OffsetCorr::getConfig() const {
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("inputs.ampl", in_ampl);
    pt.put("inputs.depth", in_depth);

    pt.put("options.ofs_0", offsets[0]);
    pt.put("options.ofs_1", offsets[1]);
    pt.put("options.ofs_2", offsets[2]);
    pt.put("options.ofs_3", offsets[3]);
    pt.put("options.ofs_4", offsets[4]);
    pt.put("options.ofs_5", offsets[5]);
    pt.put("options.ofs_6", offsets[6]);

    return pt;
}
