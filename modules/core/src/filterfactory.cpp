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

// The only filter classes core names any more (P2-4). FilterBank is not
// declared by filterfactory.hpp -- filterbank.hpp includes *this* header, not
// the other way round -- and used to arrive here transitively through one of
// the 35 filter headers this file used to carry.
#include "toffy/filterbank.hpp"
#include "toffy/parallelFilter.hpp"

// The categories libtoffy ships. This one header replaced 35 includes of
// modules that sit above core -- among them <toffy/bta/initPlugin.hpp>, the one
// module no CI runner can even build -- and the ~120-line createFilter() chain
// that needed them.
#include <toffy/filter_init.hpp>

// Still here, and still guarded, because core is what calls it: `bta` is
// registered by the factory rather than by whoever loads the library, and
// `boxes`/`toffy-oatpp` configs name `<bta>` on that basis. Moving that hook is
// `P2-10`'s (the three plugin loaders), not a side effect of this one.
#if defined(HAS_BTA)
#include <toffy/bta/initPlugin.hpp>
#endif

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

        // Core's own two creatable types, registered like everyone else's
        // (P2-4). They were the first two branches of the chain.
        registerCreator("filterBank",
                        []() -> Filter* { return new FilterBank(); });
        registerCreator("parallelFilter",
                        []() -> Filter* { return new ParallelFilter(); });

        // The categories libtoffy ships. Order matters in exactly one way:
        // registerCreator() inserts and does not overwrite, so the *first*
        // registration of a name wins. Registering the built-ins here, during
        // the first getInstance(), is what keeps a consumer's later
        // registerCreator() from replacing one -- which is what the chain
        // guaranteed by being tried before the registry. The case that does
        // change is a plug-in that registers during static initialisation,
        // before anyone calls getInstance(): it now wins over the built-in. No
        // plug-in in this tree or in either consumer does that, and
        // tests/test_filter_registry.cpp pins the later-registration case.
        initBaseFilters(*uniqueFactory);
        initCaptureFilters(*uniqueFactory);
        initSmoothingFilters(*uniqueFactory);
        initDetectionFilters(*uniqueFactory);
        initImportFilters(*uniqueFactory);
#if defined(PCL_FOUND)
        // toffy_3d does not exist when PCL is off, so neither does this
        // function. Same macro the build sets, now the only place the decision
        // is made -- the chain used to re-test it per type.
        init3dFilters(*uniqueFactory);
#endif

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

    // One path (P2-4). This used to be ~120 lines and 35 `else if (type ==
    // "...")` branches that `new`-ed every filter in the product before falling
    // through to the registry below -- which is why core included 35 headers of
    // modules above it (including the proprietary bta one), re-tested the
    // build's PCL and visualisation decisions in the C++ preprocessor, and
    // hosted a plug-in mechanism that its own filters did not use. Three of
    // those branches tested macros no build defines for core, so three compiled
    // filters could not be named from a config file at all; see
    // tests/test_filter_registry.cpp.
    //
    // Look the creator up without inserting. operator[] default-constructs a
    // missing key, so the old `creators[type]` turned every failed lookup into
    // a write to this shared registry: a typo'd filter type left a null entry
    // behind and the map grew without bound (P0-10).
    const auto it = creators.find(type);
    if (it == creators.end() || !it->second) {
        BOOST_LOG_TRIVIAL(error) << "Unknown filter: " << type;
        return nullptr;
    }

    Filter* created = it->second();
    // The creator itself may hand back null; it used to be dereferenced a few
    // lines below when the name was logged.
    if (!created) {
        BOOST_LOG_TRIVIAL(error)
            << "Creator for filter type " << type << " returned null";
        return nullptr;
    }

    // The single point where ownership starts (P2-5). From here on there is one
    // owner -- the caller -- and the index below holds a weak entry next to it,
    // so the index cannot keep the filter alive and cannot dangle. Before P2-4
    // this line also had to cover ~35 `new` expressions spread over the chain;
    // that is what made "the caller owns it" tractable in the first place, and
    // it is why the chain could be deleted without touching ownership at all.
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
