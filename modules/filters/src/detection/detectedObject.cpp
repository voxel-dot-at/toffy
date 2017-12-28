#include <boost/log/trivial.hpp>

#include "toffy/detection/detectedObject.hpp"

using namespace toffy::detection;
using namespace cv;

int DetectedObject::COUNTER = 0;

DetectedObject::DetectedObject(): first_fc(-1)
{
	id = COUNTER++;
	RNG rng(id);
	color = Scalar( rng.uniform(0,255),
			rng.uniform(0,255),
			rng.uniform(0,255) );
}

DetectedObject::DetectedObject(const DetectedObject& object): id(object.id),
    color(object.color)
{
    *this = object;
    /*contour = object.contour;
    contours = object.contours;
    hierarchy = object.hierarchy;
    mo = object.mo;
    massCenter = object.massCenter;
    idx = object.idx;
    size = object.size;

    first_fc = object.first_fc;
    first_cts = object.first_cts;
    first_ts = object.first_ts;

    fc = object.fc;
    cts = object.cts;
    ts = object.ts;

    size = object.size;*/
}

DetectedObject::~DetectedObject() {
    while (record && !record->empty()) {
        delete *record->begin();
        record->pop_front();
    }
}

DetectedObject* DetectedObject::clone(const DetectedObject& object) {
    DetectedObject *newObject = new DetectedObject(object);
    return newObject;
}

DetectedObject& DetectedObject::operator=( const DetectedObject& newDO ) {
    contour = newDO.contour;
    contours = newDO.contours;
    hierarchy = newDO.hierarchy;
    mo = newDO.mo;
    massCenter = newDO.massCenter;
    idx = newDO.idx;
    size = newDO.size;

    first_fc = newDO.first_fc;
    first_cts = newDO.first_cts;
    first_ts = newDO.first_ts;

    fc = newDO.fc;
    cts = newDO.cts;
    ts = newDO.ts;

    size = newDO.size;

    massCenter3D = newDO.massCenter3D;
    massCenterZ = newDO.massCenterZ;
    return *this;
}

void DetectedObject::init() {
    if (record) {
        BOOST_LOG_TRIVIAL(warning) << "Detected object already initialized!";
    }

    first_fc = fc;
    first_cts = cts;
    first_ts = ts;
    firstCenter = massCenter;
    size = contourArea(contour);
    record.reset(new boost::circular_buffer<detection::DetectedObject* >(10));
}

void DetectedObject::update(const DetectedObject& newDO)
{
    if (record->full()) {
        DetectedObject *last = record->back();
        *last = *this;
        record->push_front(last);
    } else
        record->push_front(new DetectedObject(*this));

    *this = newDO;
    //TODO Get new Stats here when object gets new state

    //3d points ->
    // pointTo3D(cv::Point point, float depthValue) or toffy::commons::pointTo3D(center, z, _cameraMatrix, depth->size());
}

