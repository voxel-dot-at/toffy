#ifndef __IMAGESENSOR_HPP__
#define __IMAGESENSOR_HPP__

#if defined(MSVC)
	#define DLLExport __declspec( dllexport )
    #define WIN true
#else
	#define DLLExport /**/
    #define UNIX true
#endif

#include <toffy/sensor.hpp>
#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif

class DLLExport ImageSensor: public Sensor
{

 public:
 
	virtual int start()=0;
	virtual int stop()=0;
	virtual bool isConnected() const=0;
	virtual int getFrame(cv::Mat &image)=0;

};


#endif //__IMAGESENSOR_HPP__
