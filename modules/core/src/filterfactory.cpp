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
#include "toffy/filterfactory.hpp"
//#include <toffy/toffy_config.h>

// from core/include:
#include "toffy/parallelFilter.hpp"

// from filters/include:
#include "toffy/base/amplitudeRange.hpp"
#include "toffy/base/cond.hpp"
#include "toffy/base/distAmpl.hpp"
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
#include "toffy/import/importYaml.hpp"
#include "toffy/io/csv_source.hpp"

#include "toffy/reproject/reprojectopencv.hpp"

#if defined(HAS_BTA)
#include <toffy/bta/initPlugin.hpp>
#endif

#if defined(PCL_FOUND)

#include "toffy/reproject/reprojectpcl.hpp"
#include "toffy/3d/split.hpp"
#include "toffy/3d/merge.hpp"
#include "toffy/3d/muxMerge.hpp"
#include "toffy/3d/sampleConsensus.hpp"
#include "toffy/3d/transform.hpp"
#include "toffy/3d/xyz2pcl.hpp"
#include "toffy/3d/groundprojection.hpp"
#include "toffy/viewers/exportcloud.hpp"
#include "toffy/viewers/exportYaml.hpp"
#include "toffy/detection/squareDetect.hpp"

using namespace toffy::filters::f3d;
#endif

/****** forward declarations for init functions of the single sub-libs: */
namespace toffy {
    namespace viewers {
        extern void initFilters(FilterFactory& factory);
    }
    namespace tracking {
        extern void initFilters(FilterFactory& factory);
    }
}


// TODO Create a header with a list of filters
#include <boost/log/trivial.hpp>

using namespace toffy;
using namespace toffy::filters;
using namespace std;

FilterFactory* FilterFactory::uniqueFactory;
boost::container::flat_map<std::string, Filter*> FilterFactory::_filters;

FilterFactory::FilterFactory() {}

FilterFactory* FilterFactory::getInstance()
{
    if (uniqueFactory == NULL) {
        uniqueFactory = new FilterFactory();

        // init the linked-in plugins:
#ifdef HAS_BTA
        toffy::bta::init(uniqueFactory);
#else
#warning bta missing!
#endif
        toffy::tracking::initFilters(*uniqueFactory);
        toffy::viewers::initFilters(*uniqueFactory);
    }
    return uniqueFactory;
}

FilterFactory::~FilterFactory()
{
    delete uniqueFactory;
    // TODO do I have to explicitly delete shared_ptr?
    _filters.clear();
}

int FilterFactory::deleteFilter(std::string name)
{
    boost::container::flat_map<std::string, Filter*>::iterator it;
    it = _filters.find(name);
    if (it != _filters.end()) {
        delete it->second;
        it = _filters.find(name);
        _filters.erase(it);
        return 1;
    }
    return 0;
}

Filter* FilterFactory::getFilter(const std::string& name) const
{
    try {
        return _filters.at(name);
    } catch (std::out_of_range& e) {
        BOOST_LOG_TRIVIAL(warning) << name << " not found.";
        return NULL;
    }
}

bool FilterFactory::findFilter(std::string name) const
{
    return _filters.find(name) == _filters.end() ? false : true;
}

Filter* FilterFactory::createFilter(const std::string& type,
                                    std::string /* name */)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    Filter* f;
    cout << "FF new " << type << endl;
    if (type == "filterBank")
        f = new FilterBank();
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
#endif
    else if (type == "csvSource")
        f = new capturers::CSVSource();

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
#if OPENCV_TRACKING
    else if (type == toffy::filters::smoothing::KalmanAverage::id_name)
        f = new smoothing::KalmanAverage();
#endif
    else if (type == "objectTrack")
        f = new detection::ObjectTrack();
    else if (type == "blobs")
        f = new detection::Blobs();
    else if (type == detection::BlobsDetector::id_name)
        f = new detection::BlobsDetector();
    else if (type == "focus")
        f = new Focus();
    else if (type == "mask")
        f = new detection::Mask();
    else if (type == "dataImporter")
        f = new import::DataImporter();
    else if (type == "importYaml")
        f = new import::ImportYaml();
    else if (type == "roi")
        f = new Roi();
    else if (type == BackgroundSubs::id_name)
        f = new BackgroundSubs();

#if defined(PCL_FOUND)
#ifdef PCL_VISUALIZATION
    else if (type == "reprojectpcl")
        f = new ReprojectPCL();
#endif
    else if (type == "transform")
        f = new Transform();
    else if (type == "merge")
        f = new Merge();
    else if (type == "xyz2pcl")
        f = new Xyz2Pcl();
    else if (type == "split")
        f = new Split();
    else if (type == "muxMerge")
        f = new MuxMerge();
    else if (type == "sampleConsensus")
        f = new SampleConsensus();
    else if (type == "exportcloud")
        f = new ExportCloud();
    else if (type == "exportYaml")
        f = new ExportYaml();
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
        cout << "external has " << fn << endl;
        cout << "external end" << endl;
        if (!fn) {
            BOOST_LOG_TRIVIAL(error) << "Unknown filter: " << type;
            return NULL;
        }
        f = fn();
    }
    BOOST_LOG_TRIVIAL(debug)
        << "created f->name(): " << f->name() << " id(): " << f->id()
        << "  type(): " << f->type();

    _filters.insert(std::pair<std::string, Filter*>(f->id(), f));

    return f;
}

int FilterFactory::renameFilter(Filter* f, const std::string& oldName,
                                const std::string& newName)
{
    boost::container::flat_map<std::string, toffy::Filter*>::iterator oldF =
        _filters.find(oldName);
    boost::container::flat_map<std::string, toffy::Filter*>::iterator newF =
        _filters.find(newName);

    if (newF != _filters.end()) {
        // filter with target name exists already!

        // if name == id (no name set explicitly), we fall into this case.
        // BOOST_LOG_TRIVIAL(info) << "FF we already have a filter named "
        //			   << newName << "! Abort!" <<endl;
        return 0;
    }

    if (oldF != _filters.end()) {
        // remove old instance.
        _filters.erase(oldF);
    }
    _filters.insert(std::pair<std::string, Filter*>(newName, f));

    BOOST_LOG_TRIVIAL(debug) << "FF stored " << newName << endl;
    return 1;
}

int FilterFactory::getFiltersByType(const std::string& type,
                                    std::vector<Filter*>& vec)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    for (boost::container::flat_map<std::string, Filter*>::iterator it =
             _filters.begin();
         it < _filters.end(); it++) {
        // BOOST_LOG_TRIVIAL(debug) << type;
        // BOOST_LOG_TRIVIAL(debug) << (*it)->id();
        // if ((*it)->type() == "filterBank" ||
        //	(*it)->type() == "parallelFilter")
        //{
        //     ((FilterBank *)(*it))->getFiltersByType(type, vec);
        // }
        if (it->second->type() == type) vec.push_back(it->second);
    }
    return vec.size();
}

boost::container::flat_map<std::string, CreateFilterFn> FilterFactory::creators;

void FilterFactory::registerCreator(std::string name, CreateFilterFn fn)
{
    creators.insert(std::pair<std::string, CreateFilterFn>(name, fn));
}

void FilterFactory::unregisterCreator(std::string name)
{
    creators.erase(name);
}
