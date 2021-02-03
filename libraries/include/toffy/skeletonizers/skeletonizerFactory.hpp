#ifndef SKELETONIZEFACTORY_H
#define SKELETONIZEFACTORY_H

#include <string>
#include <map>

#include <toffy/skeletonizers/skeletonizer.hpp>

typedef Skeletonizer* (*CreateSkeletonizerFn)(void);

class SkeletonizerFactory
{
private:
    SkeletonizerFactory();
    std::map<std::string,CreateSkeletonizerFn> m_FactoryMap;
public:
    ~SkeletonizerFactory();

    static SkeletonizerFactory *Get();

    void Register(const std::string &skeletonizername, CreateSkeletonizerFn pfnCreate);

    /** retrieve a skeletonizer algorithm.
     * @param name Valid options are:
     * "erodeskeletonizer","thickskeletonizer","k3mskeletonizer", "k3mppskeletonizer"
     * the last one is preferred.
     * @return a Skeletonizer instance is returned. On error, null is returned. You might need to configure it before it is usable.
     */
    Skeletonizer* getSkeletonizer(const std::string &name = "k3mppskeletonizer");
};

#endif // SKELETONIZEFACTORY_H
