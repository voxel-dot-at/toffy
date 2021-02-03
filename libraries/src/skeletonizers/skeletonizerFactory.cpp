#include "toffy/skeletonizers/skeletonizerFactory.hpp"


#include "toffy/skeletonizers/erodeSkeletonizer.hpp"
#include "toffy/skeletonizers/k3mSkeletonizer.hpp"
#include "toffy/skeletonizers/k3mppSkeletonizer.hpp"
#include "toffy/skeletonizers/k3mUpDownSkeletonizer.hpp"
#include "toffy/skeletonizers/thickSkeletonizer.hpp"
#include "toffy/skeletonizers/depthSkeletonizer.hpp"

using namespace std;

SkeletonizerFactory::SkeletonizerFactory()
{
    Register("erodeskeletonizer",&ErodeSkeletonizer::Create);
    Register("thickskeletonizer",&ThickSkeletonizer::Create);
    Register("k3mskeletonizer",&K3MSkeletonizer::Create);
    Register("k3mppskeletonizer",&K3MPPSkeletonizer::Create);
    Register("k3mupdown",&K3MUpDownSkeletonizer::Create);
    Register("depthSkeletonizer",&DepthSkeletonizer::Create);
}

SkeletonizerFactory::~SkeletonizerFactory()
{
	map<string,CreateSkeletonizerFn>::iterator it = m_FactoryMap.begin();
	
	m_FactoryMap.clear();
}

SkeletonizerFactory *SkeletonizerFactory::Get()
{
    static SkeletonizerFactory instance;
    return &instance;

}

void SkeletonizerFactory::Register(const string &skeletonizername, CreateSkeletonizerFn pfnCreate)
{
    m_FactoryMap[skeletonizername] = pfnCreate;
}

Skeletonizer *SkeletonizerFactory::getSkeletonizer(const string &skeletonizername)
{
    map<string,CreateSkeletonizerFn>::iterator it = m_FactoryMap.find(skeletonizername);
    if( it != m_FactoryMap.end() ){
        return it->second();
    }
    return 0;
}
