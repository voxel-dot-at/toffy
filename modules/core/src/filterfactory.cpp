
#include "toffy/filterfactory.hpp"
#include <toffy/toffy_config.h>

// from core/include:
#include "toffy/parallelFilter.hpp"


// from filters/include:
#include "toffy/base/amplitudeRange.hpp"
#include "toffy/base/cond.hpp"
#include "toffy/base/distAmpl.hpp"
#include "toffy/base/extractor.hpp"
#include "toffy/base/nop.hpp"
#include "toffy/base/offset.hpp"
#include "toffy/base/ofsCorr.hpp"
#include "toffy/base/polar2cart.hpp"
#include "toffy/base/range.hpp"
#include "toffy/base/rectify.hpp"
#include "toffy/base/roi.hpp"
#include "toffy/base/backgroundsubs.hpp"
#include "toffy/base/focus.hpp"

#include "toffy/smoothing/bilateral.hpp"
#include "toffy/smoothing/average.hpp"
#include "toffy/smoothing/kalmanaverage.hpp"

#include "toffy/detection/objectTrack.hpp"
#include "toffy/detection/blobs.hpp"
#include "toffy/detection/blobsDetector.hpp"
#include <toffy/detection/mask.hpp>


#include "toffy/import/dataimporter.hpp"

#include "toffy/reproject/reprojectopencv.hpp"

#include "toffy/tracking/tracker.hpp"

#ifdef WITH_VISUALIZATION
#  include "toffy/viewers/imageview.hpp"
#  include "toffy/viewers/cloudviewopencv.hpp"
#endif
#include "toffy/viewers/colorize.hpp"
#include "toffy/viewers/videoout.hpp"
//#include "toffy/exposure.hpp"


#if OCV_VERSION_MAJOR >= 3
   // OpenCV 3.x compatible filters:
#  include <toffy/tracking/cvTracker.hpp>
#endif



#if defined(PCL_FOUND)

#  ifdef PCL_VISUALIZATION
#    ifdef WITH_VISUALIZATION
#      include "toffy/viewers/cloudviewpcl.hpp"
#    endif
#    include "toffy/reproject/reprojectpcl.hpp"
#    include "toffy/3d/bbox.hpp"
#    include "toffy/3d/merge.hpp"
#    include "toffy/3d/muxMerge.hpp"
#    include "toffy/3d/sampleConsensus.hpp"
#    include "toffy/3d/transform.hpp"
#  endif
#  include "toffy/3d/groundprojection.hpp"
#  include "toffy/detection/squareDetect.hpp"

using namespace toffy::filters::f3d;
#endif

//TODO Create a header with a list of filters
#include <boost/log/trivial.hpp>

using namespace toffy;
using namespace toffy::filters;
using namespace std;

FilterFactory * FilterFactory::uniqueFactory;
boost::container::flat_map<std::string, Filter* > FilterFactory::_filters;
FilterFactory::FilterFactory(){}

FilterFactory * FilterFactory::getInstance() {
    if(uniqueFactory == NULL)
	uniqueFactory = new FilterFactory();
    return uniqueFactory;
}

FilterFactory::~FilterFactory() {
    delete uniqueFactory;
    //TODO do I have to explicitly delete shared_ptr?
    _filters.clear();
}

int FilterFactory::deleteFilter(std::string name)  {
    boost::container::flat_map<std::string, Filter* >::iterator it;
    it = _filters.find(name);
    if (it != _filters.end() ) {
	delete it->second;
	it = _filters.find(name);
	_filters.erase(it);
	return 1;
    }
    return 0;
}

Filter * FilterFactory::getFilter(const std::string& name) const {
    try {
	return _filters.at(name);
    } catch (std::out_of_range &e) {
	BOOST_LOG_TRIVIAL(warning) << name << " not found.";
	return NULL;
    }
}

bool  FilterFactory::findFilter(std::string name) const {
    return _filters.find(name) == _filters.end() ? false : true;
}


Filter * FilterFactory::createFilter(std::string type, std::string name) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    Filter * f;
    cout << "FF new " << type << endl;
    if (type == "filterBank")
	f = new FilterBank();
    else if (type == "extractor")
	f = new Extractor();
    else if (type == "offset")
	f = new OffSet();
    else if (type == "distAmpl")
	f = new DistAmpl();
    else if (type == "offsetCorr")
	f = new OffsetCorr();

    else if (type == "polar2cart")
	f = new Polar2Cart();
    else if (type == "rectify")
	f = new Rectify();

#ifdef WITH_VISUALIZATION
    else if (type == ReprojectOpenCv::id_name)
	f = new ReprojectOpenCv();
    else if (type == "cloudviewopencv")
	f = new CloudViewOpenCv();
    else if (type == ImageView::id_name)
	f = new ImageView();
#endif

    else if (type == "videoout")
	f = new VideoOut();
    //else if (type == "bta")
    //	f = new Bta();
    else if (type == "amplitudeRange")
	f = new AmplitudeRange();
    else if (type == "range")
	f = new Range();

    else if (type == "nop")
	f = new Nop();
    else if (type == "cond")
	f = new Cond();
    else if (type == "bilateral")
	f = new smoothing::Bilateral();
    else if (type == "average")
	f = new smoothing::Average();
    else if (type == toffy::filters::smoothing::KalmanAverage::id_name)
	f = new smoothing::KalmanAverage();

    else if (type == "objectTrack")
    f = new detection::ObjectTrack();
    else if (type == "blobs")
	f = new detection::Blobs();
    else if (type == detection::BlobsDetector::id_name)
	f = new detection::BlobsDetector();

    else if (type == "focus")
	f = new Focus();
    else if (type == "colorize")
	f = new Colorize();
    else if (type == "mask")
    f = new detection::Mask();
    //else if (type == "exposure")
	//f = new Exposure();


    else if (type == "dataImporter")
	f = new import::DataImporter();
    else if (type == "tracker")
	f = new tracking::Tracker();
    else if (type == "roi")
	f = new Roi();
    else if (type == BackgroundSubs::id_name)
	f = new BackgroundSubs();

#if OCV_VERSION_MAJOR >= 3
    // OpenCV 3.x compatible filters:
    else if (type == tracking::CVTracker::id_name)
	f = new tracking::CVTracker();
#endif

#if defined(PCL_FOUND)
    #ifdef PCL_VISUALIZATION
    else if (type == "reprojectpcl")
	f = new ReprojectPCL();
    #ifdef WITH_VISUALIZATION
	else if (type == "cloudviewpcl")
	    f = new CloudViewPCL();
    #endif
    else if (type == "transform")
        f = new Transform();
    else if (type == "merge")
        f = new Merge();
    else if (type == "sampleConsensus")
        f = new SampleConsensus();
    else if (type == "split")
        f = new Split();
    else if (type == "muxMerge")
        f = new MuxMerge();
    #endif
    else if (type == "groundprojection")
	f = new GroundProjection();
    else if (type == "squareDetect")
	f = new detection::SquareDetect();
#endif
//#if defined(WITH_SENSOR2D)
//    else if (type == "sensor2d")
//	f = new Sensor2();
//    else if (type == "sensorUsb")
//	f = new SensorU();
//#endif
    else if (type == "parallelFilter")
	f = new ParallelFilter();

    else {
	// try an external creator fn:
	cout << "external " << type << endl;
	CreateFilterFn fn = creators[type];
	cout << "external end"<< endl;
	if (! fn) {
	    BOOST_LOG_TRIVIAL(warning) << "Unknown filter: " << type;
	    return NULL;
	}
	f = fn();
    }
    BOOST_LOG_TRIVIAL(debug) << "f->name(): " << f->name();
    BOOST_LOG_TRIVIAL(debug) << "f->id(): " << f->id();
    BOOST_LOG_TRIVIAL(debug) << "f->type(): " << f->type();

    _filters.insert(std::pair<std::string, Filter*>(f->id(),f));

    BOOST_LOG_TRIVIAL(debug) << "Created filter: " << type;
    return f;
}

int FilterFactory::renameFilter(Filter* f, const std::string& oldName, const std::string& newName)
{
    boost::container::flat_map<std::string, toffy::Filter*>::iterator oldF = _filters.find(oldName);
    boost::container::flat_map<std::string, toffy::Filter*>::iterator newF = _filters.find(newName);

    if (newF != _filters.end() ) {
	// filter with target name exists already!

	// if name == id (no name set explicitly), we fall into this case.
	//BOOST_LOG_TRIVIAL(info) << "FF we already have a filter named "
	//			   << newName << "! Abort!" <<endl;
	return 0;
    }

    if (oldF != _filters.end() ) {
	// remove old instance.
	_filters.erase( oldF );
    }
    _filters.insert(std::pair<std::string, Filter*>(newName,f));

    BOOST_LOG_TRIVIAL(debug) << "FF stored " << newName << endl;
    return 1;
}

int FilterFactory::getFiltersByType(const std::string& type,
				    std::vector<Filter *> &vec)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    for (boost::container::flat_map<std::string, Filter* >::iterator it=_filters.begin();
	 it < _filters.end(); it++)
    {
	//BOOST_LOG_TRIVIAL(debug) << type;
	//BOOST_LOG_TRIVIAL(debug) << (*it)->id();
	//if ((*it)->type() == "filterBank" ||
	//	(*it)->type() == "parallelFilter")
	//{
	//    ((FilterBank *)(*it))->getFiltersByType(type, vec);
	//}
	if (it->second->type() == type)
	    vec.push_back(it->second);
    }
    return vec.size();
}

boost::container::flat_map<std::string, CreateFilterFn> FilterFactory::creators;

void FilterFactory::registerCreator( std::string name, CreateFilterFn fn) 
{ 
    creators.insert( std::pair<std::string, CreateFilterFn>(name, fn) );
}

void FilterFactory::unregisterCreator(std::string name) { 
    creators.erase(name);
}

