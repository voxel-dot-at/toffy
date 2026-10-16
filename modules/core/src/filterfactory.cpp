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
boost::container::flat_map<std::string, std::weak_ptr<Filter> >
    FilterFactory::_filters;

FilterFactory::FilterFactory() {}

FilterFactory* FilterFactory::getInstance()
{
    if (uniqueFactory == NULL) {
        uniqueFactory = new FilterFactory();

        // init the linked-in plugins:
#ifdef HAS_BTA
        toffy::bta::init(uniqueFactory);
#endif
        // No BTA is a supported configuration, not a code smell: the build already
        // says so via `message(WARNING "no bta library!")` in CMakeLists.txt. A
        // #warning here fired on every compile of the default configuration and
        // trained everyone to ignore build output.
        toffy::tracking::initFilters(*uniqueFactory);
        toffy::viewers::initFilters(*uniqueFactory);
    }
    return uniqueFactory;
}

FilterFactory::~FilterFactory()
{
    // `delete uniqueFactory` used to be the first line here. It is a singleton
    // deleting itself from inside its own destructor: ~FilterFactory would be
    // entered again, on the same object, forever. Nothing in the tree deletes
    // the factory, so the function was unreachable -- but "unreachable" is not
    // "harmless" in the one place the class claims to own memory (P2-5).
    //
    // Clearing the index cannot destroy anything either way: the entries are
    // weak (P2-5), so whatever outlives the factory is owned elsewhere.
    _filters.clear();
}

int FilterFactory::deleteFilter(std::string name)
{
    // Forget the name; do not destroy anything. The filter dies with the last
    // FilterPtr holding it, which is the FilterBank it is pipelined into
    // (P2-5). The old version did `delete it->second` here, which is what made
    // the factory a second owner of every filter in the tree and made
    // ~FilterBank's delete-by-name teardown unsafe.
    //
    // An entry whose owner has already gone is not a filter: it is reported as
    // not found, like findFilter() reports it, and is erased on the way out so
    // the index does not accumulate corpses.
    boost::container::flat_map<std::string, std::weak_ptr<Filter> >::iterator it =
        _filters.find(name);
    if (it == _filters.end()) return 0;

    const bool alive = !it->second.expired();
    _filters.erase(it);
    return alive ? 1 : 0;
}

Filter* FilterFactory::getFilter(const std::string& name) const
{
    boost::container::flat_map<std::string, std::weak_ptr<Filter> >::const_iterator
        it = _filters.find(name);
    if (it == _filters.end()) {
        BOOST_LOG_TRIVIAL(warning) << name << " not found.";
        return NULL;
    }

    // lock(), not get(): an expired entry names a filter that no longer exists.
    // Returning it used to be possible -- the registry kept the pointer after
    // the object was deleted -- and is the use-after-free this change removes.
    const FilterPtr f = it->second.lock();
    if (!f) {
        BOOST_LOG_TRIVIAL(warning) << name << " not found.";
        return NULL;
    }
    return f.get();
}

bool FilterFactory::findFilter(std::string name) const
{
    boost::container::flat_map<std::string, std::weak_ptr<Filter> >::const_iterator
        it = _filters.find(name);
    return it != _filters.end() && !it->second.expired();
}

FilterPtr FilterFactory::createFilter(const std::string& type,
                                      std::string /* name */)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    // Raw on purpose, and only until the FilterPtr is built at the bottom of
    // this function: the chain below is ~35 `new` expressions, and taking
    // ownership in exactly one place is what makes "the caller owns it" true.
    Filter* created = nullptr;
    if (type == "filterBank")
        created = new FilterBank();
    else if (type == "offset")
        created = new OffSet();
    else if (type == "distAmpl")
        created = new DistAmpl();
    else if (type == "offsetCorr")
        created = new OffsetCorr();

    else if (type == "polar2cart")
        created = new Polar2Cart();
    else if (type == "rectify")
        created = new Rectify();

#ifdef WITH_VISUALIZATION
    else if (type == ReprojectOpenCv::id_name)
        created = new ReprojectOpenCv();
#endif
    else if (type == "csvSource")
        created = new capturers::CSVSource();

    else if (type == "amplitudeRange")
        created = new AmplitudeRange();
    else if (type == "range")
        created = new Range();

    else if (type == "nop")
        created = new Nop();
    else if (type == "cond")
        created = new Cond();
    else if (type == "bilateral")
        created = new smoothing::Bilateral();
    else if (type == "average")
        created = new smoothing::Average();
#if OPENCV_TRACKING
    else if (type == toffy::filters::smoothing::KalmanAverage::id_name)
        created = new smoothing::KalmanAverage();
#endif
    else if (type == "objectTrack")
        created = new detection::ObjectTrack();
    else if (type == "blobs")
        created = new detection::Blobs();
    else if (type == detection::BlobsDetector::id_name)
        created = new detection::BlobsDetector();
    else if (type == "focus")
        created = new Focus();
    else if (type == "mask")
        created = new detection::Mask();
    else if (type == "dataImporter")
        created = new import::DataImporter();
    else if (type == "importYaml")
        created = new import::ImportYaml();
    else if (type == "roi")
        created = new Roi();
    else if (type == BackgroundSubs::id_name)
        created = new BackgroundSubs();

#if defined(PCL_FOUND)
#ifdef PCL_VISUALIZATION
    else if (type == "reprojectpcl")
        created = new ReprojectPCL();
#endif
    else if (type == "transform")
        created = new Transform();
    else if (type == "merge")
        created = new Merge();
    else if (type == "xyz2pcl")
        created = new Xyz2Pcl();
    else if (type == "split")
        created = new Split();
    else if (type == "muxMerge")
        created = new MuxMerge();
    else if (type == "sampleConsensus")
        created = new SampleConsensus();
    else if (type == "exportcloud")
        created = new ExportCloud();
    else if (type == "exportYaml")
        created = new ExportYaml();
    else if (type == "groundprojection")
        created = new GroundProjection();
    else if (type == "squareDetect")
        created = new detection::SquareDetect();
#endif
    //#if defined(WITH_SENSOR2D)
    //    else if (type == "sensor2d")
    //	created = new Sensor2();
    //    else if (type == "sensorUsb")
    //	created = new SensorU();
    //#endif
    else if (type == "parallelFilter")
        created = new ParallelFilter();

    else {
        // Look the creator up without inserting. operator[] default-constructs
        // a missing key, so the old `creators[type]` turned every failed lookup
        // into a write to this shared registry: a typo'd filter type left a
        // null entry behind and the map grew without bound.
        const auto it = creators.find(type);
        if (it == creators.end() || !it->second) {
            BOOST_LOG_TRIVIAL(error) << "Unknown filter: " << type;
            return nullptr;
        }
        created = it->second();
        // The creator itself may hand back null; it used to be dereferenced a
        // few lines below when the name was logged.
        if (!created) {
            BOOST_LOG_TRIVIAL(error)
                << "Creator for filter type " << type << " returned null";
            return nullptr;
        }
    }

    // The single point where ownership starts (P2-5). From here on there is one
    // owner -- the caller -- and the registry below holds a weak index entry
    // next to it, so the index cannot keep the filter alive and cannot dangle.
    const FilterPtr f(created);

    BOOST_LOG_TRIVIAL(debug)
        << "created f->name(): " << f->name() << " id(): " << f->id()
        << "  type(): " << f->type();

    _filters.insert(
        std::pair<std::string, std::weak_ptr<Filter> >(f->id(), f));

    return f;
}

int FilterFactory::renameFilter(const FilterPtr& f, const std::string& oldName,
                                const std::string& newName)
{
    // A weak index entry can only be copied from a shared reference, hence the
    // FilterPtr parameter: with a raw Filter* this function would have had to
    // build a control block around an object that already has one, which is a
    // double delete waiting for the second owner to appear.
    if (!f) return 0;

    boost::container::flat_map<std::string, std::weak_ptr<Filter> >::iterator oldF =
        _filters.find(oldName);
    boost::container::flat_map<std::string, std::weak_ptr<Filter> >::iterator newF =
        _filters.find(newName);

    if (newF != _filters.end()) {
        if (!newF->second.expired()) {
            // filter with target name exists already!

            // if name == id (no name set explicitly), we fall into this case.
            // BOOST_LOG_TRIVIAL(info) << "FF we already have a filter named "
            //			   << newName << "! Abort!" <<endl;
            return 0;
        }
        // The name is free: whatever held it is gone. Reuse the slot rather
        // than letting an expired entry block the name forever.
        _filters.erase(newF);
    }

    if (oldF != _filters.end()) {
        // remove old instance.
        _filters.erase(oldF);
    }
    _filters.insert(
        std::pair<std::string, std::weak_ptr<Filter> >(newName, f));

    BOOST_LOG_TRIVIAL(debug) << "FF stored " << newName << endl;
    return 1;
}

int FilterFactory::getFiltersByType(const std::string& type,
                                    std::vector<Filter*>& vec)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    for (boost::container::flat_map<std::string, std::weak_ptr<Filter> >::iterator
             it = _filters.begin();
         it != _filters.end(); it++) {
        // BOOST_LOG_TRIVIAL(debug) << type;
        // BOOST_LOG_TRIVIAL(debug) << (*it)->id();
        // if ((*it)->type() == "filterBank" ||
        //	(*it)->type() == "parallelFilter")
        //{
        //     ((FilterBank *)(*it))->getFiltersByType(type, vec);
        // }
        // lock() rather than dereferencing the map value: an expired entry is
        // not a filter of any type. (`it < _filters.end()` used to be the loop
        // condition; flat_map iterators are random access, but != is the form
        // that survives the container being swapped out from under us.)
        const FilterPtr f = it->second.lock();
        if (f && f->type() == type) vec.push_back(f.get());
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

bool FilterFactory::hasCreator(const std::string& name)
{
    // find(), never operator[]: this is a query and must not create an entry.
    return creators.find(name) != creators.end();
}
