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

// Included directly because this header uses them: <memory> for the FilterPtr /
// weak_ptr pair below, flat_map for both registries. Before P2-5 the flat_map
// came in transitively through filter.hpp -> frame.hpp, which is the kind of
// dependency that turns into a build break in someone else's translation unit.
#include <memory>
#include <string>

#include <boost/container/flat_map.hpp>

#include "toffy/filter.hpp"

// DLLExport removed -- TOFFY_EXPORT (from CMake's generate_export_header) is the
// real export macro, and it was already the one used on FilterFactory below.

namespace toffy
{

/**
 * @brief type signature for functions returning new a filter of an
 * (unknown) type.
 *
 * Deliberately still a raw pointer, and deliberately not a FilterPtr
 * (P2-5): this is the plug-in ABI. A compiled .so exports
 * `init(FilterFactory*)` and hands one of these to registerCreator(), and
 * the calling convention of a function returning a shared_ptr is not the
 * same as one returning a pointer, so changing it would break every plug-in
 * built against an older libtoffy for no ownership benefit - the pointer
 * comes back from `new` with exactly one owner, and createFilter() wraps it
 * on the way out. See the X6 decision in
 * cleanup/plan/controller-extraction.md.
 *
 * A creator therefore *hands over* what it returns: it must be a filter that
 * `new` has just produced, the creator must not keep a copy of it, and it must
 * not be deleted by anyone once createFilter() has taken it. Returning a
 * pointer to something still owned elsewhere gives that owner a filter which
 * can be destroyed under its feet by the pipeline.
 */
typedef Filter* (*CreateFilterFn)(void);

/**
 * @brief Singleton factory that keep track of all filters
 * @ingroup Core
 *
 * Provides a filter container that makes all filters be unique identify
 * It should be the only way to create any kind of filter with the single
 * exception of the Base FilterBank that is declared in the Player.
 *
 * Ownership (P2-5): the factory creates filters and *indexes* them; it does
 * not own them. createFilter() hands back the only strong reference, and the
 * registry below holds weak ones, so a filter lives exactly as long as the
 * FilterBank (or caller) that holds it and its index entry expires with it.
 * Nothing in the tree deletes a Filter any more; `deleteFilter()` is the
 * request to forget a name, not to destroy anything, and it is no longer
 * needed for teardown - a bank that drops its filters drops the objects.
 *
 * Before P2-5 the registry held raw pointers and deleteFilter() deleted
 * through them, which made the factory a second owner of every filter and
 * made ~FilterBank's name-keyed teardown a use-after-free waiting to happen
 * (findings C1/A17 in cleanup/findings/ownership.md).
 *
 * FilterFactory is also responsible for add external filter to toffy. It
 * provides a creator container that allows to include filters in runtime.
 * The creators should be register with the type name of the filter.
 *
 */
class TOFFY_EXPORT FilterFactory
{

public:
    /**
     * @brief Singleton accesor
     * @return FilterFactory *
     */
    static FilterFactory * getInstance();

    /**
     * @brief Delete all filters
     *
     */
    virtual ~FilterFactory();

    /**
     * @brief Single way to create filters and keep track of them
     * @param type of filter
     * @param name identifier
     * @return FilterPtr, empty if the type is unknown or the creator failed
     *
     * The returned FilterPtr is the filter's *only* strong reference: whoever
     * creates a filter owns it, and it dies when that owner lets go. The
     * factory keeps a weak index entry so getFilter(name) can find it while
     * it lives (P2-5).
     */
    FilterPtr createFilter(const std::string& type, std::string name="");

    /**
     * @brief Forget the filter registered under this name
     * @param name
     * @return 1 if a live filter was registered under that name, 0 otherwise
     *
     * Does not destroy anything. The filter is destroyed when the last
     * FilterPtr referring to it goes away, which is normally the FilterBank
     * that holds it in its pipeline. Calling this is now optional at
     * teardown; it stays for callers that want the name released early.
     */
    int deleteFilter(std::string name);


    /**
     * @brief Change the filter checking that it is unique
     * @param f
     * @param oldName
     * @param newName
     * @return Positive on success, negative or 0 if failed
     *
     * Takes the filter by FilterPtr, not by Filter*: the registry holds weak
     * references, and a weak reference can only be made from a shared one.
     * Building a weak_ptr from a raw Filter* would give the object a second
     * control block and two owners deleting it, which is the bug this whole
     * change is removing.
     */
    int renameFilter(const FilterPtr& f, const std::string& oldName, const std::string& newName);

    /**
     * @brief Access to filters globaly
     * @param name
     * @return Filter *, or nullptr if the name is unknown or its owner is gone
     *
     * The result is a non-owning observer: it is valid while somebody else
     * (a FilterBank's pipeline, or a FilterPtr of your own) owns the filter.
     * Keep a FilterPtr if you need to hold it across a remove().
     */
    Filter * getFilter(const std::string& name) const;

    /**
     * @brief Check if a filter exist
     * @param name
     * @return True if a filter with that name is alive, false otherwise
     *
     * "Exist" means registered *and* still owned by something: an index entry
     * whose owner has gone is not a filter and is reported as missing.
     */
    bool  findFilter(std::string name) const;

    /**
     * @brief Get a list of all filter of a type
     * @param type
     * @param vec [out] The list of found filters
     * @return Positive number of found filters, negative or 0 if none found
     */
    int getFiltersByType(const std::string& type,
			 std::vector<Filter *> &vec);

    /**
     * @brief Register a filter creation function with a name.
     * @param name
     * @param fn
     */
    static void registerCreator( std::string name, CreateFilterFn fn);


    /**
     * @brief Unregister a filter creation function
     * @param name
     */
    static void unregisterCreator(std::string name);

    /**
     * @brief Check whether a creator function is registered for a type.
     * @param name the filter type name
     * @return true if registerCreator() has registered this name
     *
     * Read-only query on the creator registry. It exists because a lookup must
     * not go through operator[]: that default-constructs a missing key, so a
     * failed lookup would insert a null entry into shared global state. See
     * FilterFactory::createFilter().
     */
    static bool hasCreator(const std::string& name);

    void clearCreators() {creators.clear();}

private:

    static FilterFactory *uniqueFactory; ///< Singleton
    /**
     * @brief Index of every filter the factory has created, by id or name.
     *
     * Weak on purpose (P2-5): the pipeline owns, this looks up. An entry
     * whose filter has been destroyed expires instead of dangling, so the
     * lookups above can answer "no such filter" instead of handing back a
     * pointer to freed memory.
     */
    static boost::container::flat_map<std::string, std::weak_ptr<Filter> > _filters;
    FilterFactory(); ///< Singleton

    /**
     * @brief Contains a list of filter creators which could be modify runtime.
     */
    static boost::container::flat_map<std::string, CreateFilterFn> creators;
};
}
