#pragma once


#include "toffy/filter.hpp"
#include <fstream>

/**
 * @brief
 *
 */
namespace toffy {
namespace detection {
/**
 * @brief A try of base class for all kind of detector.
 * @ingroup Detection
 *
 * This class was thought as base class for object detector. The detection will
 * algorithms will be pretty similar and just the shape and/or characteristics
 * of the target will change from one kind the object to the other.
 * (I.e. hand/arm, leg, head, etc)
 *
 * Image filtering and noise removals will be almost the same.
 *
 * @todo This is not been used yet
 *
 */
class Mask : public Filter {
public:

    static const std::string id_name; ///< Filter identifier

    Mask();

    virtual ~Mask() {}

    //virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);


    boost::shared_ptr<cv::Mat> depth, ampl, proj2d, fground, new_mask, img3d, _cameraMatrix;
    double maxSizeX , maxSizeY, fovx, fovy;
    double _dis, _apertureWidth, _apertureHeight;
    int _scale;
    bool _creation;

    //void createCloud();
    void generate3D();
    //void groundProjection();
    void groundProjection2();
    void projectBack();
    void amplitudeMasking();
    void morfExMask(cv::Mat &mask);
    void fillMaskGaps();

private:
    std::string _in_depth, _in_ampl, _out_mask;
    std::string _maskPath, _in_cameraMatrix;
    static std::size_t _filter_counter;
};
}}
