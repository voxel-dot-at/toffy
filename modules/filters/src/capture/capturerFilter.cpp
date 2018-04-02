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
#include <map>
#include <ctime>
#include <iostream>

#ifdef MSVC
    #define _USE_MATH_DEFINES
    #include <cmath>
#endif

#include <boost/log/trivial.hpp>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/range/algorithm_ext/erase.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "toffy/toffy_config.h"
#include "toffy/common/filenodehelper.hpp"

#include "toffy/capture/capturerFilter.hpp"

using namespace toffy;
using namespace toffy::capturers;
using namespace cv;
namespace fs = boost::filesystem;

//std::size_t Bta::_filter_counter = 1;

boost::property_tree::ptree CapturerFilter::getConfig() const
{
    boost::property_tree::ptree pt;
    //boost::property_tree::ptree bta;
    //std::cout << "bta::getConfig "<<endl;

    pt = Filter::getConfig();

    pt.put("options.flip_x", _flip_x);
    pt.put("options.flip_y", _flip_y);
    pt.put("options.flip", _flip);
    pt.put("options.center_position", _center_position);
    pt.put("options.rotations", _rotations);
    //pt.put("options.flip_x", _rotations);
    pt.put("options.startIdx", _cnt);

    return pt;
}

bool CapturerFilter::tsd() const
{
    return _tsd;
}

void CapturerFilter::setTsd(bool tsd)
{
    _tsd = tsd;
}

cv::Point3d CapturerFilter::center_position() const
{
    return _center_position;
}

void CapturerFilter::setCenter_position(const cv::Point3d &center_position)
{
    _center_position = center_position;
}

cv::Point3d CapturerFilter::rotations() const
{
    return _rotations;
}

void CapturerFilter::setRotations(const cv::Point3d &rotations)
{
    _rotations = rotations;
}

bool CapturerFilter::backward() const
{
    return _backward;
}

void CapturerFilter::setBackward(bool backward)
{
    _backward = backward;
}

bool CapturerFilter::flip_x() const
{
    return _flip_x;
}

void CapturerFilter::setFlip_x(bool flip_x)
{
    _flip_x = flip_x;
}

bool CapturerFilter::flip_y() const
{
    return _flip_y;
}

void CapturerFilter::setFlip_y(bool flip_y)
{
    _flip_y = flip_y;
}
CapturerFilter::CapturerFilter(): _cnt(0), _beginFile(0), _endFile(0),
    _playBack(false), _save(false), _flip(false), _flip_x(false),
    _flip_y(false), _tsd(true), _backward(false),
    _center_position(0, 0., 0.) , _rotations(0.,.0,.0)
{
}

CapturerFilter::CapturerFilter(std::string type,
    std::size_t counter): Filter(type,counter), _cnt(0), _beginFile(0),
    _endFile(0), _playBack(false), _save(false), _flip(false), _flip_x(false),
    _flip_y(false), _tsd(true),
    _backward(false), _center_position(0, 0., 0.) , _rotations(0.,.0,.0)
{
}

void CapturerFilter::savePath(const std::string &newPath) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    if (newPath.empty()) {
	_savePath = newPath;
	return;
    }
    fs::path fsPath(newPath);
    if ( !fs::exists(fsPath)) {
	BOOST_LOG_TRIVIAL(warning) << "Given path " << newPath
				   << " does not exist.";
	return;
    }


    if ( !fs::is_directory(fsPath)) {
	BOOST_LOG_TRIVIAL(warning) << "Given path " << newPath
				   << " is not a directory."
				   << " Removing file name to get the path";
	fsPath = fsPath.parent_path();
    }

    _savePath = newPath;
}

void CapturerFilter::setLoadPath(const std::string &newPath) {
    _loadPath = newPath;
}

void CapturerFilter::setSavePath(const std::string &newPath) {
    _savePath = newPath;
}

int CapturerFilter::loadPath(const std::string &newPath) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    fs::path fsPath(newPath);
    if ( !fs::exists(fsPath)) {
	BOOST_LOG_TRIVIAL(warning) << "Given path " << newPath
				   << " does not exist.";
	return -1;
    }


    if ( !fs::is_directory(fsPath)) {
	BOOST_LOG_TRIVIAL(warning) << "Given path " << newPath
				   << " is not a directory."
				   << " Removing file name to get the path";
	fsPath = fsPath.parent_path();
    }

    _loadPath = newPath;

    fs::directory_iterator end_iter;
    _fileExt = ".r";
    typedef std::multimap<int, fs::path> result_set_t;
    result_set_t result_set;
    for( fs::directory_iterator dir_iter(fsPath) ; dir_iter != end_iter ; ++dir_iter) {
	if (fs::is_regular_file(dir_iter->status()) &&
		dir_iter->path().extension() == _fileExt)
	{
	    result_set.insert(result_set_t::value_type(boost::lexical_cast<int>(dir_iter->path().stem().c_str()), *dir_iter));
	}
    }
    if (result_set.size() == 0) {
	_fileExt = ".rw";
	for( fs::directory_iterator dir_iter(fsPath) ; dir_iter != end_iter ; ++dir_iter) {
	    if (fs::is_regular_file(dir_iter->status()) &&
		    dir_iter->path().extension() == _fileExt)
	    {
		result_set.insert(result_set_t::value_type(boost::lexical_cast<int>(dir_iter->path().stem().c_str()), *dir_iter));
	    }
	}
    }
    if (!result_set.empty()) {
	_loadPath = newPath;
	//BOOST_LOG_TRIVIAL(debug) << (*result_set.begin()).first;
	//BOOST_LOG_TRIVIAL(debug) << (*result_set.rbegin()).first;
	_beginFile = (*result_set.begin()).first;
	_endFile = (*result_set.rbegin()).first;
	//BOOST_LOG_TRIVIAL(debug) << _beginFile;
	//BOOST_LOG_TRIVIAL(debug) << _endFile;
	return result_set.size();
    }
    return -1;
}

void CapturerFilter::save(const bool &save) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    if (save == true) {
	long int _saveTimeStamp;
	#if defined(MSVC)
	    using namespace boost::posix_time;
	    ptime pt(second_clock::local_time());
	    static ptime epoch(boost::gregorian::date(1970, 1, 1));
	    time_duration diff(pt - epoch);
	    _saveTimeStamp= diff.ticks() / diff.ticks_per_second();
	#else
	    time(&_saveTimeStamp);
	#endif
	fs::path newPath(_savePath);
	//newPath /= name();
	//string _strPath = savePath + string("/") + id() + string("/") + boost::lexical_cast<std::string>(_saveTimeStamp);
	if (!_saveFolder.empty())
	    newPath /= _saveFolder;
	if (_tsd)
	    newPath /= boost::lexical_cast<std::string>(_saveTimeStamp);
	newPath /= name();
	BOOST_LOG_TRIVIAL(debug) << newPath.string();
	try {
	    fs::create_directories(fs::absolute(newPath));
	} catch(const fs::filesystem_error& e) {
	    BOOST_LOG_TRIVIAL(warning) << "Could not create folder: "
				       << _strPath << "; Reason: "
				       << e.code().message();
	    _save = false;
	    return;
	}
	_strPath = newPath.string();
	_cnt=0;
    }
    _save = save;
}

void CapturerFilter::playback(const bool &pb) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    if (pb == true)
	_cnt = _beginFile;
    _playBack = pb;
}

void CapturerFilter::updateConfig(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

    using namespace boost::property_tree;

    //Filter::setCommon(pt);

    Filter::updateConfig(pt);

    _loadPath = pt.get<std::string>("options.loadPath",_loadPath);
    boost::trim(_loadPath);
    std::cout << "LP:::::: [[" << _loadPath << "]] " << std::endl;
    loadPath(_loadPath);
    _savePath = pt.get<std::string>("options.savePath",_savePath);
    boost::trim(_savePath);

    _flip = pt.get<bool>("options.flip",_flip);
    _flip_x = pt.get<bool>("options.flip_x",_flip_x);
    _flip_y = pt.get<bool>("options.flip_y",_flip_y);
    _tsd = pt.get<bool>("options.timeStamped",_tsd);

    _cnt = pt.get<int>("options.startIdx",_cnt);

    try {
	boost::optional<const boost::property_tree::ptree& > ocvo = pt.get_child_optional( "options.center_position" );
	if (ocvo.is_initialized()) {
	    boost::property_tree::ptree os;
	    std::string tmp = ocvo.get().get_value("center_position");
	    boost::remove_erase_if(tmp, boost::is_any_of("[,]"));
	    os.put("center_position",tmp);
	    cv::FileStorage fs = commons::loadOCVnode(os);
	    fs.getFirstTopLevelNode() >> _center_position;
	    fs.release();
	} else
	    BOOST_LOG_TRIVIAL(debug) << "Node options.center_position not found.";


	ocvo = pt.get_child_optional( "options.rotations" );
	if (ocvo.is_initialized()) {
	    boost::property_tree::ptree os;
	    std::string tmp = ocvo.get().get_value("rotations");
	    boost::remove_erase_if(tmp, boost::is_any_of("[,]"));
	    os.put("rotations",tmp);
	    cv::FileStorage fs = commons::loadOCVnode(os);
	    fs.getFirstTopLevelNode() >> _rotations;
	    fs.release();
	} else
	    BOOST_LOG_TRIVIAL(debug) << "Node options.rotations not found.";


	_playBack = pt.get<bool>("options.playback",_playBack);
	if (_playBack) { this->playback(_playBack);   }

    } catch (const std::exception& ex) {
	BOOST_LOG_TRIVIAL(debug) << "Camera pose definition values wrong. Ex:" << ex.what();
	return;
    }

    }


// TODO MOVE TO SOMEWHERE ELSE
static inline double rad2deg(double rad){return rad*(180/M_PI);}//Convert radians to degrees
static inline double deg2rad(double deg){return deg*(M_PI/180);}//Convert degrees to radians


/** moused together from the PCL implmementation:

inline float rad2deg (float alpha)
{
    return (alpha * 57.29578f);
}

inline float deg2rad (float alpha)
{
    return (alpha * 0.017453293f);
}

template <typename Scalar> void getTransformation (Scalar x, Scalar y, Scalar z, 
						   Scalar roll, Scalar pitch, Scalar yaw, 
						   Eigen::Transform<Scalar, 3, Eigen::Affine> &t)
{
    Scalar A = cos (yaw),  B = sin (yaw),  C  = cos (pitch), D  = sin (pitch),
	E = cos (roll), F = sin (roll), DE = D*E,         DF = D*F;

    t (0, 0) = A*C;  t (0, 1) = A*DF - B*E;  t (0, 2) = B*F + A*DE;  t (0, 3) = x;
    t (1, 0) = B*C;  t (1, 1) = A*E + B*DF;  t (1, 2) = B*DE - A*F;  t (1, 3) = y;
    t (2, 0) = -D;   t (2, 1) = C*F;         t (2, 2) = C*E;         t (2, 3) = z;
    t (3, 0) = 0;    t (3, 1) = 0;           t (3, 2) = 0;           t (3, 3) = 1;
}
*/

void CapturerFilter::setCamera2Wcs(toffy::Frame& out, std::string name) 
{
    boost::shared_ptr<Eigen::Affine3f> f2t(new Eigen::Affine3f());
    //Eigen::Affine3f t;

    Eigen::Matrix3f m;
    m = Eigen::AngleAxisf(deg2rad(rotations().x), Eigen::Vector3f::UnitX())
	    * Eigen::AngleAxisf(deg2rad(rotations().y), Eigen::Vector3f::UnitY())
	    * Eigen::AngleAxisf(deg2rad(rotations().z), Eigen::Vector3f::UnitZ());

    Eigen::Translation<float,3> t(center_position().x,
				center_position().y,
				center_position().z);

    /*getTransformation<float> (center_position().x,
			      center_position().y,
			      center_position().z,
			      deg2rad(rotations().x),
			      deg2rad(rotations().y),
			      deg2rad(rotations().z), 
			      t);*/

    //TODO right order
    *f2t = m*t;
    out.addData(name,f2t);
}
