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
#ifndef __M100WRAPPER_H__
#define __M100WRAPPER_H__

#if defined(MSVC)
#define DLLExport __declspec( dllexport )
#define WIN true
#define UNIX false
#define RAWFILE ".rw"
#else
#define DLLExport /**/
#define UNIX true
#define WIN false
#define RAWFILE ".r"
#endif

#include <bta.h>
#include <boost/property_tree/ptree.hpp>
#include <toffy/io/imagesensor.hpp>

struct network {
    std::string tcp_ip,udp_ip;
    short tcp_port,udp_port;
};

class DLLExport BtaWrapper : public ImageSensor
{
public:

    BtaWrapper();

    virtual ~BtaWrapper();

    virtual int start() {return connect();}
    virtual int stop() {return disconnect();}

    virtual int getFrame(cv::Mat & ){return -1;}

    virtual int parseConfig(std::string fileConfig);
    int parseConfig(const boost::property_tree::ptree pt);
    virtual int connect();
    int reConnect();
    virtual int disconnect();
    virtual bool isConnected() const;
    virtual int capture(char * &buffer);
    virtual int registerOp(unsigned int reg);
    virtual int registerOp(unsigned int reg, unsigned int data);
    int startGrabbing(std::string filename);
    int stopGrabbing();

    virtual int getLibParam(int &param, float &data);
    virtual int setLibParam(int &param, float &data);

    //virtual unsigned short getImgType();
    //virtual int getData(unsigned char* data, int size);
    virtual int getDistances(float * &depth, int &size, char *data);
    virtual int getAmplitudes(unsigned short * &amplitudes, int &size, char *data);

    virtual int getDisSize(char *data, int &x, int &y);
    virtual int getAmpSize(char *data, int &x, int &y);

    virtual int getFrameTime(char *data, unsigned int &timeStamp);
    virtual int getFrameCounter(char *data, unsigned int &counter);
    virtual int getFrameRef(char *data, unsigned int &mf, unsigned int &it);

    virtual char * loadFrame(char *data, std::string ext);
    virtual char * serializeFrame(char *data, size_t &size);
    virtual int freeFrame(char *data);

    virtual unsigned int getIntegrationTime();
    virtual float getFrameRate();
    virtual unsigned long getModulationFrequency();

    virtual int setIntegrationTime(unsigned int it);
    virtual int setFrameRate(float fr);
    virtual int setModulationFrequency(unsigned long mf);

    int getTemps(char *data, float &mt, float &lt, float &gt);
    int getMainTemp(char *data, float &mt);
    int getLedTemp(char *data, float &lt);
    int getGenericTemp(char *data, float &gt);

    virtual int reset();

    //void deserializeFrame(char *data, size_t &size);
    int saveRaw(std::string fileName, char *data);
    char * loadRaw(std::string rawFile);

    std::string getBltstream() const;
    void setBltstream(const std::string &value);

    BTA_DeviceType getDeviceType() const;
    void setDeviceType(const BTA_DeviceType &value);

    bool isAsync() { return async; }
    void waitForFrame(); // wait for next frame to arrive....

    // queue handling:

private:
    BTA_Config config;
    BTA_Handle handle;
    BTA_Status status;
    BTA_DeviceInfo *deviceInfo;

    //variables needed to fill pointer in BTA_Config
    uint8_t udpDataIpAddr[6];
    uint8_t tcpDeviceIpAddr[6];
    uint8_t udpControlOutIpAddr[6];
    uint8_t udpControlInIpAddr[6];
    std::string calibFileName,
    uartPortName,
    bltstreamFilename;
#if defined(BTA_P100)
    static const int retries = 2;
#else
    static const int retries = 10;
#endif
    bool async; //< set to true if frameArrived* callbacks are used.

    unsigned int manufacturer, device;

    int width, height;
};

#endif
