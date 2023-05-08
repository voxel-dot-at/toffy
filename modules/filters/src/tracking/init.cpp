#include <iostream>

#include <toffy/filterfactory.hpp>

#if OPENCV_TRACKING
#  include <toffy/tracking/cvTracker.hpp>
#endif
#include <toffy/tracking/tracker.hpp>

namespace toffy {

namespace tracking {

toffy::Filter* CreateTracker(void)
{
    return new Tracker();
}

#if OPENCV_TRACKING
toffy::Filter* CreateCVTracker(void)
{
    return new CVTracker();
}
#endif

void initFilters(FilterFactory& factory)
{
    using namespace std;
    cout << "toffy::tracking::initFilters()" << endl;
#if OPENCV_TRACKING
    factory.registerCreator(CVTracker::id_name, CreateCVTracker);
#endif
    factory.registerCreator(Tracker::id_name, CreateTracker);
}

}  // namespace viewers
}  // namespace toffy
