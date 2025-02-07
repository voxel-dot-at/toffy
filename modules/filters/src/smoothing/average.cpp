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

#include <boost/algorithm/string/case_conv.hpp>

#include "toffy/filter_helpers.hpp"
#include "toffy/smoothing/average.hpp"

using namespace toffy;
using namespace toffy::filters::smoothing;
using namespace cv;
using namespace std;
namespace logging = boost::log;

std::size_t Average::_filter_counter = 1;
const std::string Average::id_name = "average";

Average::Average() :Filter(Average::id_name,_filter_counter),
			_in_img("depth"),
			_out_img("depth"),
			_size(10)
{
    _filter_counter++;
}


bool Average::filter(const Frame &in, Frame& out)
{
    LOG(debug) << __FUNCTION__ <<  " " << id();

    matPtr img;
    matPtr new_img;

    try {
    	img = boost::any_cast<matPtr >(in.getData(_in_img));

    } catch(const boost::bad_any_cast &) {
        LOG(warning) <<
            "Could not cast input " << _in_img <<
            ", filter  " << id() <<" not applied.";
        return false;
    }
    try {
        if (out.hasKey(_out_img)) {
            new_img = boost::any_cast<matPtr >(out.getData(_out_img));
        } else {
            LOG(info) << "init new_img!";
            new_img.reset(new Mat());
        }
    } catch(const boost::bad_any_cast &) {
        LOG(warning) <<
            "Could not cast output " << _in_img <<
            ", filter  " << id() <<" not applied.";
        return false;
    }

    //bilateralFilter(*depth, *bi, d, sigmaColor, sigmaSpace, BORDER_REPLICATE);

    matPtr imgadd;
    if (_queue.size() == _size) {
        LOG(debug) << "queue FULL";
        imgadd = _queue.front();
        _queue.pop_front();
        img->copyTo(*imgadd);
        _queue.push_back(imgadd);
    } else {
        LOG(debug) << "Filling queue";
        imgadd.reset( new Mat(img->clone()));
        _queue.push_back(imgadd);
        LOG(debug) << _queue.size();
    }

    if (img->type() != _dst.type() || img->size() != _dst.size()) {
        _dst.release();
        _dst = Mat(img->size(), img->type());
    }
    if (img->size() != _cnt.size()) {
        _cnt.release();
        _cnt = Mat(img->size(), CV_32S);
    }
    _dst = Scalar::all(0.0);
    _cnt = Scalar::all(0);

    //int count[size];
    //memset(count, 0, sizeof count);
    float* src;
    float* dst;
    int* cnt;
    for (size_t i=0; i < _queue.size(); i++) {
	for(int row = 0; row < _queue[i]->rows; ++row) {
	    src = _queue[i]->ptr<float>(row);
	    dst = _dst.ptr<float>(row);
	    cnt = _cnt.ptr<int>(row);

	    for(int col = 0; col < _queue[i]->cols; ++col) {
		if (src[col] > 0.0) {
			dst [col] += src[col];
			cnt[col]++;
		}
	    }
	}
    }

    for(int row = 0; row < _dst.rows; ++row) {
	dst = _dst.ptr<float>(row);
        cnt = _cnt.ptr<int>(row);

        for(int col = 0; col < _dst.cols; ++col) {
            if (cnt[col] != 0)
                dst [col] /= cnt[col];

        }
    }

    *new_img = _dst.clone();



    /**new_img = _queue[0]->clone();
    for (size_t i=1; i < _queue.size(); i++)
	*new_img += *_queue[i];

    BOOST_LOG_TRIVIAL(debug) << "averaging over: " <<_queue.size();
    *new_img /= _queue.size();*/

    
    out.addData(_out_img, new_img);
    //cout << id() << " wrote to " << _out_img << endl;

    return true;
}

/*
int Average::loadConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();
    //const boost::property_tree::ptree& average = pt.get_child(type());

    updateConfig(pt.begin()->second );
    return 1;
}
*/

boost::property_tree::ptree Average::getConfig() const
{
    boost::property_tree::ptree pt;
    pt = Filter::getConfig();
    pt.put("options.size", _size);

    pt.put("inputs.img", _in_img);

    pt.put("outputs.img", _out_img);

    return pt;
}

void Average::updateConfig(const boost::property_tree::ptree &pt) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    Filter::updateConfig(pt);
    _size = pt.get("options.size", _size);
    _queue.clear();

    _in_img = pt.get("inputs.img", _in_img);

    _out_img = pt.get("outputs.img", _out_img);

    LOG(debug) << "averaging " << _size << " " << _in_img << " -> " << _out_img;
}
