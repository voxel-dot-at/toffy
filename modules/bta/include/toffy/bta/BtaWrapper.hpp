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

#include <boost/thread.hpp>
#include <boost/property_tree/ptree.hpp>

#include <bta.h>
#include <toffy/io/imagesensor.hpp>

struct network {
    std::string tcp_ip,udp_ip;
    short tcp_port,udp_port;
};

const int MAX_CHANNEL_SELECTIONS=8;


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
    virtual float getGlobalOffset();

    virtual int setIntegrationTime(unsigned int it);
    virtual int setFrameRate(float fr);
    virtual int setModulationFrequency(unsigned long mf);
    virtual int setGlobalOffset(float ofsInMm);

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

    void saveAmplCSV(const std::string& fileName);
    void saveDistCSV(const std::string& fileName);


    bool isAsync() { return async; }
    BTA_Frame* waitForNextFrame(); // wait for next frame to arrive....

    // queue handling:
    void updateFrame(BTA_Frame* frame); // update the 

    bool hasChannels; // set to true if channels have been selected this way
    std::string channelSelectionName(int i) { 
        return (i>=0 && i<MAX_CHANNEL_SELECTIONS) ? chanSelectionName[i] : "invalid";
        }

    /** call BTA_SetChannelSelection with the current set of channels */
    BTA_Status setChannels();

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
    unsigned int manufacturer, device;

    bool async; //< set to true if frameArrived* callbacks are used.

    boost::mutex frameMutex, fillFrameMutex;
    boost::condition_variable newFrameCond;

    BTA_Frame* frames[2]; //< array of frames, one is filled by bta lib, the other in use
    BTA_Frame* frameInUse; // pointer to the current frame
    BTA_Frame* frameToFill; // pointer to the current frame
    int toFillIndex; // index of the current frame
    bool hasBeenUpdated; // do we have new data yet?

    int numChannels; // number of active channels
    BTA_ChannelSelection channels[MAX_CHANNEL_SELECTIONS];
    std::string chanSelectionName[MAX_CHANNEL_SELECTIONS];
    std::string theChannels;  //< comma-separated list of channels as read by xml
    bool parseChannelSelection(const std::string& chans);

    /** change frames, resets hasBeenUpdated
     * @return the new frame to use for processing (frameInUse)
    */
    BTA_Frame* flipFrame();

};

/** get the name of the data type for a channel id */
extern std::string getChannelTypeName(BTA_ChannelId cid);

#endif
