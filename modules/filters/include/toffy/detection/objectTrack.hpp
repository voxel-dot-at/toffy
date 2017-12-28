#pragma once


#include <toffy/filter.hpp>
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
class ObjectTrack : public Filter {
public:

    static const std::string id_name; ///< Filter identifier

    ObjectTrack();

    virtual ~ObjectTrack() {}

    //virtual int loadConfig(const boost::property_tree::ptree& pt);
    virtual boost::property_tree::ptree getConfig() const;
    void updateConfig(const boost::property_tree::ptree &pt);

    virtual bool filter(const Frame& in, Frame& out);
private:
    std::string _in_img,
        _in_vec, _out_vec;
    static std::size_t _filter_counter;
    int cnt;
    int numObjects;
    std::string updateScript;

    void publishCount();
};
}}
