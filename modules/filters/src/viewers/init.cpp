#include <toffy/filterfactory.hpp>

#include <toffy/viewers/cloudviewopencv.hpp>
// The includes have to be conditional in the same way the registrations below
// already are: cloudviewpcl.hpp and exportcloud.hpp pull in pcl headers, and this
// file is compiled whether or not PCL is enabled. Guarding only the registerCreator
// calls still left the headers being parsed.
#if PCL_VIZ
#include <toffy/viewers/cloudviewpcl.hpp>
#endif
#include <toffy/viewers/colorize.hpp>
#if PCL_FOUND
#include <toffy/viewers/exportcloud.hpp>
#endif
#include <toffy/viewers/exportcsv.hpp>
#include <toffy/viewers/imageview.hpp>
#include <toffy/viewers/videoout.hpp>
#include <iostream>

namespace toffy {

toffy::Filter* CreateColorize(void)
{
    return new Colorize();
}

toffy::Filter* CreateImageView(void)
{
    return new ImageView();
}

toffy::Filter* CreateVideoOut(void)
{
    return new VideoOut();
}

#if PCL_FOUND
toffy::Filter* CreateExportCloud(void)
{
    return new ExportCloud();
}
#endif
toffy::Filter* CreateExportCSV(void)
{
    return new ExportCSV();
}

#if OPENCV_VIZ
toffy::Filter* CreateCloudViewOpenCv(void)
{
    return new CloudViewOpenCv();
}
#endif

#if PCL_VIZ
toffy::Filter* CreateCloudViewPCL(void)
{
    return new CloudViewPCL();
}
#endif

namespace viewers {

void initFilters(FilterFactory& factory)
{
    using namespace std;
    cout << "toffy::viewers::initFilters()" << endl;
#if OPENCV_VIZ
    factory.registerCreator("cloudviewopencv", CreateCloudViewOpenCv);
#endif
#if PCL_VIZ
    factory.registerCreator("cloudviewpcl", CreateCloudViewPCL);
#endif
    factory.registerCreator("colorize", CreateColorize);
#if PCL_FOUND
    factory.registerCreator("exportcloud", CreateExportCloud);
#endif
    factory.registerCreator("exportcsv", CreateExportCSV);
    factory.registerCreator("imageview", CreateImageView);
    factory.registerCreator("videoout", CreateVideoOut);
}

}  // namespace viewers
}  // namespace toffy
