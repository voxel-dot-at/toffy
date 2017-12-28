#pragma once

#include "toffy/filter.hpp"

#ifdef MSVC
#define DLLExport __declspec( dllexport )
#else
#define DLLExport /**/
#endif

namespace toffy
{

/**
 * @brief type signature for functions returning new a filter of an
 * (unknown) type.
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
 * Should also be the single way to delete filters.
 *
 * FilterFactory is also responsible for add external filter to toffy. It
 * provides a creator container that allows to include filters in runtime.
 * The creators should be register with the type name of the filter.
 *
 */
class /*DLLExport*/ TOFFY_EXPORT FilterFactory
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
     * @return Filter *
     */
    Filter *createFilter(std::string type, std::string name="");

    /**
     * @brief Remove filter
     * @param name
     * @return
     */
    int deleteFilter(std::string name);


    /**
     * @brief Change the filter checking that it is unique
     * @param f
     * @param oldName
     * @param newName
     * @return Positive on success, negative or 0 if failed
     */
    int renameFilter(Filter* f, const std::string& oldName, const std::string& newName);

    /**
     * @brief Access to filters globaly
     * @param name
     * @return Filter *
     */
    Filter * getFilter(const std::string& name) const;

    /**
     * @brief Check if a filter exist
     * @param name
     * @return True if found, false if filter does not exist
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

    void clearCreators() {creators.clear();}

private:

    static FilterFactory *uniqueFactory; ///< Singleton
    static boost::container::flat_map<std::string, Filter* > _filters; ///< Filters container
    FilterFactory(); ///< Singleton

    /**
     * @brief Contains a list of filter creators which could be modify runtime.
     */
    static boost::container::flat_map<std::string, CreateFilterFn> creators;
};
}
