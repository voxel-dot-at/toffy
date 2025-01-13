/*
   Copyright 2012-2021 Simon Vogl <svogl@voxel.at> VoXel Interaction Design - www.voxel.at

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
     * "erodeskeletonizer","thickskeletonizer","k3mskeletonizer", "k3mppskeletonizer", "k3mupdown"
     * the last one is preferred.
     * @return a Skeletonizer instance is returned. On error, null is returned. You might need to configure it before it is usable.
     */
    Skeletonizer* getSkeletonizer(const std::string &name = "k3mupdown");
};

#endif // SKELETONIZEFACTORY_H
