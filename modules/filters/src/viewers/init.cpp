#include <toffy/filterfactory.hpp>

#include <toffy/viewers/cloudviewopencv.hpp>
#include <toffy/viewers/cloudviewpcl.hpp>
#include <toffy/viewers/colorize.hpp>
#include <toffy/viewers/exportcloud.hpp>
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

toffy::Filter* CreateExportCloud(void)
{
    return new ExportCloud();
}
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
    factory.registerCreator("exportcloud", CreateExportCloud);
    factory.registerCreator("exportcsv", CreateExportCSV);
    factory.registerCreator("imageview", CreateImageView);
    factory.registerCreator("videoout", CreateVideoOut);
}

}  // namespace viewers
}  // namespace toffy
