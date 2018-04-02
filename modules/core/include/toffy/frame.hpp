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

#include <toffy/toffy_export.h>
#include <toffy/toffy_config.h>
#include <vector>
#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/container/flat_map.hpp>

#ifdef MSVC
#define DLLExport __declspec( dllexport )
#else
#define DLLExport /**/
#endif

namespace cv {
class Mat;
}

namespace toffy
{
/**
 * @brief Share pointer to cv::Mat
 */
typedef boost::shared_ptr<cv::Mat> matPtr;

/**
 * @brief Frame is a container where any filter could add, retrieve, modify and
 *  delete all kind of data.
 * @ingroup Core
 *
 * The Frame is a miscellanea container from where the filters get dynamic data
 *  runtime. Is the only interface the filter can deal with. It keeps trace the
 *  single instances of the data and assure that they are uniquely identify.
 *
 * A toffy application does contain  a single frame from where all its filter
 *  will manage the data.
 *
 */
class TOFFY_EXPORT Frame {
public:

    Frame();
    /**
     * @brief Copy constructor
     * @param f
     */
    Frame(const Frame& f);
    virtual ~Frame();

    /**
     * @brief operator =
     * @param x
     * @return
     */
    Frame& operator= (const Frame& x) {data = x.data; return *this;}

    /**
     * @brief Check if a data key is in the frame
     * @param key
     * @return True if found, false if not
     */
    bool hasKey(std::string key) const;

    /**
     * @brief Insert data in the Frame with the given key. If the key does
     *	already exist, overwrite the data
     * @param key
     * @param in
     */
    void addData(std::string key, boost::any in);

    /**
     * @brief Delete data from the Frame with key. If not found does not do
     *  anything
     * @param key
     * @return True if deleted, False if failed
     */
    bool removeData(std::string key);

    /**
     * @brief Clean all data in Frame
     */
    void clearData();

    //std::vector<std::string> keys() const;

    //boost::any getData(std::string key) const;

    /**
     * @brief Get data from Frame with key
     * @param key
     * @return Data if key found, empty in not.
     */
    boost::any getData(const std::string& key) const;

    /**
     * @brief Shorted getter for booleans in frame
     * @param key
     * @return
     */
    inline bool getBool(const std::string& key) const;

    /**
     * @brief Shorted getter for unsigned integers in frame
     * @param key
     * @return
     */
    inline unsigned int getUInt(const std::string& key) const;

    /**
     * @brief Shorted getter for integers in frame
     * @param key
     * @return
     */
    inline int getInt(const std::string& key) const;

    /**
     * @brief Shorted getter for doubles in frame
     * @param key
     * @return
     */
    inline double getDouble(const std::string& key) const;

    /**
     * @brief Shorted getter for floads in frame
     * @param key
     * @return
     */
    inline float getFloat(const std::string& key) const;

    /**
     * @brief Shorted getter for matPtr in frame
     * @param key
     * @return
     */
    inline matPtr getMatPtr(const std::string& key) const;

    /**
     * @brief Shorted getter for std::string values in frame
     * @param key
     * @return
     */
    inline std::string getString(const std::string& key) const;

private:
    /**
     * Data container in frame.
     *
     * Its has a unique string key and uses boost::any to save any kind
     * of data.
     * Use boost::shared_ptr to avoid any memory leak.
     */
    boost::container::flat_map< std::string, boost::any > data;
};

inline unsigned int Frame::getUInt(const std::string& key) const {
    unsigned int u = boost::any_cast<unsigned int>( getData(key) );
    return u;
}

inline bool Frame::getBool(const std::string& key) const {
    bool b = boost::any_cast<bool>( getData(key) );
    return b;
}

inline int Frame::getInt(const std::string& key) const {
    int u = boost::any_cast<int>( getData(key) );
    return u;
}

inline double Frame::getDouble(const std::string& key) const {
    double d = boost::any_cast<double>( getData(key) );
    return d;
}

inline float Frame::getFloat(const std::string& key) const {
    float f = boost::any_cast<float>( getData(key) );
    return f;
}

inline matPtr Frame::getMatPtr(const std::string& key) const {
    matPtr m = boost::any_cast<matPtr>( getData(key) );
    return m;
}

inline std::string Frame::getString(const std::string& key) const {
    std::string s = boost::any_cast<std::string>( getData(key) );
    return s;
}

}
