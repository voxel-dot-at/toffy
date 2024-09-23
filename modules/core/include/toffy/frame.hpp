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

#include <vector>

#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/container/flat_map.hpp>

#include <opencv2/core.hpp>

#include <toffy/toffy_export.h>

#ifdef MSVC
#define DLLExport __declspec(dllexport)
#else
#define DLLExport /**/
#endif

namespace cv {
class Mat;
}

namespace toffy {
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
class TOFFY_EXPORT Frame
{
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
    Frame& operator=(const Frame& x)
    {
        data = x.data;
        return *this;
    }

    typedef enum
    {
        NotFound,
        Any,
        Bool,
        Int,
        Uint,
        Double,
        Float,
        Mat,
        String
    } SlotDataType;

    typedef struct
    {
        std::string key;
        SlotDataType dt;
        std::string description;
    } SlotInfo;

    /**
     * @brief Check if a data key is in the frame
     * @param key
     * @return True if found, false if not
     */
    bool hasKey(std::string key) const;

    void info(std::vector<SlotInfo>& fields) const;

    /**
     * @brief Insert data in the Frame with the given key. If the key does
     *	already exist, overwrite the data
     * @param key string representation of the data
     * @param v the value
     * @param dt dataType
     * @param description an (optional) description, esp. for Any data types
     */
    void addData(std::string key, boost::any v, SlotDataType dt,
                 const std::string& description);

    void addData(std::string key, boost::any v, SlotDataType dt);

    void addData(std::string key, boost::any v) { addData(key, v, Any); };

    void addData(std::string key, matPtr m) { addData(key, m, Mat); }

    void addData(std::string key, bool v) { addData(key, v, Bool); }

    void addData(std::string key, int v) { addData(key, v, Int); }

    void addData(std::string key, long v) { addData(key, v, Int); }

    void addData(std::string key, unsigned int v) { addData(key, v, Uint); }

    void addData(std::string key, unsigned long v) { addData(key, v, Uint); }

    void addData(std::string key, float v) { addData(key, v, Float); }

    void addData(std::string key, double v) { addData(key, v, Double); }

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

    SlotDataType getDataType(const std::string& key) const
    {
        return meta.find(key) != meta.end() ? meta.find(key)->second : NotFound;
    }

    std::string getDescription(const std::string& key) const
    {
        return desc.find(key) != desc.end() ? desc.find(key)->second : "";
    }

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

    // optional variant - returns dfault value if key not present:
    inline bool optBool(const std::string& key, bool dfault) const;
    inline unsigned int optUInt(const std::string& key,
                                unsigned int dfault) const;
    inline int optInt(const std::string& key, int dfault) const;
    inline double optDouble(const std::string& key, double dfault) const;
    inline float optFloat(const std::string& key, float dfault) const;
    inline matPtr optMatPtr(const std::string& key, matPtr dfault) const;
    inline std::string optString(const std::string& key,
                                 std::string& dfault) const;

    // insGet: get a value, if key key does not exist insert it
    // inline bool optBool(const std::string& key, bool dfault) const;
    // inline unsigned int optUInt(const std::string& key, unsigned int dfault) const;
    // inline int optInt(const std::string& key, int dfault) const;
    // inline double optDouble(const std::string& key, double dfault) const;
    // inline float optFloat(const std::string& key, float dfault) const;
    // inline matPtr optMatPtr(const std::string& key, matPtr dfault) const;
    // inline std::string optString(const std::string& key, std::string& dfault) const;

    /*
    // set :
    inline bool setGetBool(const std::string& key, bool dfault);
    inline unsigned int setGetUInt(const std::string& key, unsigned int dfault);
    inline int setGetInt(const std::string& key, int dfault);
    inline double setGetDouble(const std::string& key, double dfault);
    inline float setGetFloat(const std::string& key, float dfault);
    inline matPtr setGetMatPtr(const std::string& key, Size size, int type);
    inline std::string setGetString(const std::string& key, std::string& dfault);
*/
    /** get matPtr, if not set, create and insert a new one: */
    inline matPtr getSertMatPtr(const std::string& key, cv::Size size,
                                int type);

   private:
    /**
     * Data container in frame.
     *
     * Its has a unique string key and uses boost::any to save any kind
     * of data.
     * Use boost::shared_ptr to avoid any memory leak.
     */
    boost::container::flat_map<std::string, boost::any> data;

    /** data type of the slot */
    boost::container::flat_map<std::string, SlotDataType> meta;

    /** optional description for a data slot */
    boost::container::flat_map<std::string, std::string> desc;
};

inline unsigned int Frame::getUInt(const std::string& key) const
{
    unsigned int u = boost::any_cast<unsigned int>(getData(key));
    return u;
}

inline bool Frame::getBool(const std::string& key) const
{
    bool b = boost::any_cast<bool>(getData(key));
    return b;
}

inline int Frame::getInt(const std::string& key) const
{
    int u = boost::any_cast<int>(getData(key));
    return u;
}

inline double Frame::getDouble(const std::string& key) const
{
    double d = boost::any_cast<double>(getData(key));
    return d;
}

inline float Frame::getFloat(const std::string& key) const
{
    float f = boost::any_cast<float>(getData(key));
    return f;
}

inline matPtr Frame::getMatPtr(const std::string& key) const
{
    matPtr m = boost::any_cast<matPtr>(getData(key));
    return m;
}

inline std::string Frame::getString(const std::string& key) const
{
    std::string s = boost::any_cast<std::string>(getData(key));
    return s;
}

// optional variant - returns dfault value if key not present:
inline bool Frame::optBool(const std::string& key, bool dfault) const
{
    return hasKey(key) ? getBool(key) : dfault;
};
inline unsigned int Frame::optUInt(const std::string& key,
                                   unsigned int dfault) const
{
    return hasKey(key) ? getUInt(key) : dfault;
};
inline int Frame::optInt(const std::string& key, int dfault) const
{
    return hasKey(key) ? getInt(key) : dfault;
};
inline double Frame::optDouble(const std::string& key, double dfault) const
{
    return hasKey(key) ? getDouble(key) : dfault;
};
inline float Frame::optFloat(const std::string& key, float dfault) const
{
    return hasKey(key) ? getFloat(key) : dfault;
};

inline matPtr Frame::optMatPtr(const std::string& key, matPtr dfault) const
{
    return hasKey(key) ? getMatPtr(key) : dfault;
};

inline std::string Frame::optString(const std::string& key,
                                    std::string& dfault) const
{
    return hasKey(key) ? getString(key) : dfault;
};

inline matPtr Frame::getSertMatPtr(const std::string& key, cv::Size size,
                                   int type)
{
    if (!hasKey(key)) {
        matPtr mp(new cv::Mat(size, type));
        addData(key, mp, Mat);
    }
    return getMatPtr(key);
}

}  // namespace toffy
