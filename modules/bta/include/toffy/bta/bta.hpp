#pragma once

#include <toffy/capture/capturerFilter.hpp>
#include <toffy/bta/BtaWrapper.hpp>

namespace toffy {
namespace capturers {

/**
 * @brief Bluetechnix BtaTofApi wrapper filter
 * @ingroup Capturers
 *
 * For detailled information see \ref bta_page description page
 *
 */
class Bta : public CapturerFilter {

    BtaWrapper *sensor; ///< Sensor
    bool bta_stream; ///< Flag indicates if using btastream files
    static std::size_t _filter_counter; ///< Internal class counter

public:
    static const std::string id_name; ///< Filter identifier

    /**
     * @brief Bta
     */
    Bta();

    /**
     * @brief ~Bta
     */
    virtual ~Bta();

    /**
     * @brief Accesss to the camera wrapper
     * @return BtaWrapper *
     *
     */
    BtaWrapper * getSensor() const {return sensor;}

    virtual int loadConfig(const boost::property_tree::ptree& pt);

    virtual boost::property_tree::ptree getConfig() const;

    virtual void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);

    virtual int connect();
    virtual int disconnect();
    virtual bool isConnected();

    virtual bool playback() const {return CapturerFilter::playback();}
    virtual void playback(const bool &pb);
    virtual int loadPath(const std::string &newPath);
    virtual void save(const bool &save);
    virtual void savePath(const std::string &newPath);

    static Filter* creator() {return new Bta();}

    //TODO MOVE TO OPENCV
    //virtual void setCamera2Wcs(toffy::Frame& out, std::string name);

private:
    // frame size data
    int distsSize;
    int width, height;
    std::string _out_depth, _out_ampl,
        _out_mf, _out_it,
        _out_fc, _out_ts,
        _out_mt, _out_lt, _out_gt; ///< Input image name
};

}}
