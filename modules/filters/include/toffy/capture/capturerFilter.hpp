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
#pragma once

#include "toffy/filter.hpp"
#include <toffy/imagesensor.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif

#ifdef MSVC
#define DLLExport __declspec( dllexport )
#define RAWFILE ".rw"
#else
#define DLLExport /**/
#define RAWFILE ".r"
#endif

/** @defgroup Capturers Capturers
 *
 * Capturers module
 *
 */

//! Base toffy namespace
namespace toffy {
//! Base capturers namespace
namespace capturers {
/**
 * @brief Base class for any image capturer.
 * @ingroup Capturers
 *
 * Implement base behavior like setting paths for loading or reading
 * frames, pose and image flip.
 *
 */
class /*DLLExport*/ TOFFY_EXPORT CapturerFilter : public Filter {
    //Sensor *sensor;
    int _cnt, ///< counter for load and save
	_beginFile, ///< start frame for playback frames
	_endFile; ///< last frame playback
    std::string _loadPath, ///< Folder or file to load frames
	_savePath, ///<File prefix or file name
	_saveFolder; ///< Extra folder path where to save frames
    bool _playBack, ///< Flag
	_save, ///< Flag
	_flip, ///< Flag for flipping both x and y
	_flip_x, ///< Flag
	_flip_y, ///< Flag
	_tsd, ///< Flag for saving file in a timestamped subfolder
	_backward; ///< Flag playback
    std::string _strPath, ///< Keep the full path to loaded or saved path
	_fileExt; ///< TODO Extension of the frame file to save/load
    cv::Point3d _center_position, ///< Position of the sensor w.r.t the wcs
	_rotations; ///< Rotation w.r.t the wcs
protected:

    /**
     * @brief CapturerFilter
     *
     * Only can be created by derived capturer classes
     *
     */
    CapturerFilter();

    /**
     * @brief CapturerFilter
     * @param type
     * @param counter
     *
     * Extra constructor with Filter fashion
     */
    CapturerFilter(std::string type, std::size_t counter);

public:

    /**
     * @brief ~CapturerFilter
     */
    virtual ~CapturerFilter(){}

    //Todo sensor should be a base class for all cameras
    //Sensor * getSensor() const {return sensor;}

    /**
     * @brief loadConfig
     * @param pt
     * @return Positive when success, negative or 0 when failled
     *
     * Base config loader for capturers
     *
     */
    //virtual int loadConfig(const boost::property_tree::ptree& pt);

    /**
     * @brief getConfig
     * @return ptree
     */
    virtual boost::property_tree::ptree getConfig() const;

    /**
     * @brief updateConfig
     * @param pt
     */
    virtual void updateConfig(const boost::property_tree::ptree &pt);

    /**
     * @brief connect
     * @return
     *
     *
     */
    virtual int connect() =0;

    /**
     * @brief disconnect
     * @return Positive when success, negative or 0 when failed
     *
     */
    virtual int disconnect()=0;

    /**
     * @brief isConnected
     * @return True when connected, false not connected.
     */
    virtual bool isConnected()=0;

    /**
     * @brief Create an publish to a frame the camera to wcs transform
     * @param out
     * @param name
     *
     */
    virtual void setCamera2Wcs(toffy::Frame& out, std::string name);

    /**
     * @brief Check if path is correct and set it
     * @param newPath
     * @return Positive when success, negative or 0 when failed
     *
     * Check if path is well formed and exist
     */
    virtual int loadPath(const std::string &newPath);

    /**
     * @brief Set the path without checking
     * @param newPath
     */
    void setLoadPath(const std::string &newPath);

    /**
     * @brief Getter loadPath
     * @return std::string
     */
    virtual std::string loadPath() const {return _loadPath;}

    /**
     * @brief Check and from the savePath
     * @param newPath
     *
     * Check if path is well formed and exist
     *
     */
    virtual void savePath(const std::string &newPath);

    /**
     * @brief Set savePath without checking
     * @param newPath
     */
    void setSavePath(const std::string &newPath);

    /**
     * @brief getSavePath
     * @return std::string
     */
    std::string getSavePath() const {return _savePath;}

    /**
     * @brief Getter saveFolder
     * @return std::string
     */
    std::string saveFolder() const {return _saveFolder;}

    /**
     * @brief Setter saveFolder
     * @param newFolder
     */
    void saveFolder(std::string newFolder) {_saveFolder = newFolder;}

    /**
     * @brief Getter fileExt
     * @return std::string
     */
    std::string fileExt() const {return _fileExt;}

    /**
     * @brief Setter fileExt
     * @param newFileExt
     */
    void fileExt(std::string newFileExt) {_fileExt = newFileExt;}

    /**
     * @brief Getter strPath
     * @return std::string
     */
    std::string strPath() const {return _strPath;}

    /**
     * @brief Setter strPath
     * @param newStrPath
     */
    void strPath(std::string newStrPath) {_strPath = newStrPath;}

    /**
     * @brief Getter playback
     * @return std::string
     */
    virtual bool playback() const {return _playBack;}

    /**
     * @brief Setter playback
     * @param pb
     */
    virtual void playback(const bool &pb);

    /**
     * @brief Getter save
     * @return bool
     */
    bool save() const {return _save;}

    /**
     * @brief Setter setSave
     * @param save
     */
    void setSave(const bool &save) {_save = save;}

    virtual void save(const bool &save); //TODO ???

    /**
     * @brief Getter timeStamped
     * @return boll
     */
    bool timeStamped() const {return _tsd;}

    /**
     * @brief Setter timeStamped
     * @param tsd
     */
    void timeStamped(bool tsd) {_tsd = tsd;}

    /**
     * @brief Getter cnt
     * @return int
     */
    int cnt() const {return _cnt;}

    /**
     * @brief Setter cnt
     * @param cnt
     */
    void cnt(int cnt) {_cnt = cnt;}

    /**
     * @brief Getter beginFile
     * @return int
     */
    int beginFile() const {return _beginFile;}

    /**
     * @brief Setter beginFile
     * @param beginFile
     */
    void beginFile(int beginFile) {_beginFile = beginFile;}

    /**
     * @brief Getter endFile
     * @return int
     */
    int endFile() const {return _endFile;}

    /**
     * @brief setter endFile
     * @param endFile
     */
    void endFile(int endFile) {_endFile = endFile;}

    /**
     * @brief Getter flip
     * @return bool
     */
    bool flip() const {return _flip;}

    /**
     * @brief Setter flip
     * @param val
     */
    void flip(bool val) {_flip = val;}

    /**
     * @brief Setter tsd
     * @return bool
     */
    bool tsd() const;

    /**
     * @brief setTsd
     * @param tsd
     */
    void setTsd(bool tsd);

    /**
     * @brief Getter center_position
     * @return cv::Point3d
     */
    cv::Point3d center_position() const;

    /**
     * @brief setCenter_position
     * @param center_position
     */
    void setCenter_position(const cv::Point3d &center_position);

    /**
     * @brief Getter rotations
     * @return cv::Point3d
     *
     * x,y,z rotations in degrees roll, pitch and yaw
     */
    cv::Point3d rotations() const;

    /**
     * @brief setRotations
     * @param rotations
     *
     * x,y,z rotations in degrees roll, pitch and yaw
     */
    void setRotations(const cv::Point3d &rotations);

    /**
     * @brief Getter backward
     * @return bool
     */
    bool backward() const;

    /**
     * @brief setBackward
     * @param backward
     */
    void setBackward(bool backward);

    /**
     * @brief Getter flip_x
     * @return
     */
    bool flip_x() const;

    /**
     * @brief setFlip_x
     * @param flip_x
     */
    void setFlip_x(bool flip_x);

    /**
     * @brief Getter flip_y
     * @return
     */
    bool flip_y() const;

    /**
     * @brief setFlip_y
     * @param flip_y
     */
    void setFlip_y(bool flip_y);
};
}
}
