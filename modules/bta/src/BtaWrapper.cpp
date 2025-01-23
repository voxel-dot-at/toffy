#include <arpa/inet.h>  // inet_aton
#include <string.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

#include <bta.h>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/thread.hpp>

#include <toffy/bta/BtaWrapper.hpp>
#include <toffy/bta/FrameHeader.hpp>
#include <toffy/filter_helpers.hpp>

using namespace std;

static void BTA_CALLCONV infoEventCbEx2(BTA_Handle /*handle*/,
                                        BTA_Status status, int8_t *msg,
                                        void * /*userArg*/)
{
    // BtaWrapper* bta = (BtaWrapper*)userArg;
    if (status == BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(debug)
            << "   BTACallback: infoEventEx2 (" << status << ") " << msg;
    } else {
        BOOST_LOG_TRIVIAL(warning)
            << "   BTACallback: infoEventEx2 (" << status << ") " << msg;
    }
}

static void BTA_CALLCONV frameArrivedEx2(
    BTA_Handle /*handle*/, BTA_Frame *frame, void *arg,
    struct BTA_FrameArrivedReturnOptions * /*frameArrivedReturnOptions*/)
{
    if (!frame) {
        BOOST_LOG_TRIVIAL(warning)
            << "   BTACallback: frameArrivedEx2 NO FRAME ";
        return;
    }
    /*
    BOOST_LOG_TRIVIAL(debug)
        << "   BTACallback: frameArrivedEx2 (" << frame->frameCounter << ") ";
*/
    BtaWrapper *bta = (BtaWrapper *)arg;
    /*
    int musec, msec, sec, min, hours;

    unsigned long timeArr = frame->timeStamp;
    musec = timeArr % 1000;
    timeArr /= 1000;
    msec = timeArr % 1000;
    timeArr /= 1000;
    sec = timeArr % 60;
    timeArr /= 60;
    min = timeArr % 60;
    hours = timeArr / 60;

    printf("Frame arrived: Frame no. %d at %02d:%02d:%02d.%03d %03d.\n",
           frame->frameCounter, hours, min, sec, msec, musec);
*/
    bta->updateFrame(frame);
    return;
}

static void errorHandling(BTA_Status status)
{
    if (status != BTA_StatusOk) {
        char statusString[100] = {
            0,
        };
        BTAstatusToString(status, statusString, sizeof(statusString));
        BOOST_LOG_TRIVIAL(warning)
            << "bta: " << statusString << ". error id: " << status;
    }
}

std::string BtaWrapper::getBltstream() const { return bltstreamFilename; }

void BtaWrapper::setBltstream(const std::string &value)
{
    bltstreamFilename = value;
}

BtaWrapper::BtaWrapper()
    : manufacturer(1), device(0), async(false), state(disconnected)
{
    handle = 0;
    deviceInfo = NULL;
    // TODO is size really needed;
    // setSize(160, 120);
    // TODO create header with manufacturer codes
    frames[0] = new BTA_Frame();
    frames[1] = new BTA_Frame();
    frames[2] = new BTA_Frame();
    frameToFill = frames[0];
    frameInUse = frames[1];
    toFillIndex = 0;
    hasBeenUpdated = false;

    manufacturer = 1;
}

BtaWrapper::~BtaWrapper() { disconnect(); }

int BtaWrapper::parseConfig(string configFile)
{
    boost::property_tree::ptree pt;
    cout << endl
         << endl
         << "---------------------------------- " << endl
         << endl;
    try {
        boost::property_tree::read_xml(configFile, pt);
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(warning)
            << "Could not open config file: " << configFile << ". " << e.what();
        return -1;
    }
    BOOST_LOG_TRIVIAL(debug) << "Config file opened.";
    boost::optional<boost::property_tree::ptree &> opt =
        pt.get_child_optional("opencv_storage");
    if (opt.is_initialized()) {
        return parseConfig(opt.get());
    } else
        return parseConfig(pt);
}

int BtaWrapper::parseConfig(const boost::property_tree::ptree pt)
{
    BTAinitConfig(&config);
    bool pres;

    pres = pt_optional_get_default<uint8_t>(pt, "connection.shmDataEnabled",
                                            config.shmDataEnabled, 0);
    if (pres) {
        BOOST_LOG_TRIVIAL(debug)
            << "shmDataEnabled is set! now=" << (int)config.shmDataEnabled;
    }

    // n.b. we DON't set the default mcast channel 224.0.0.1 ;
    // that way pres is only set when udpDataIp is really present in the ptree
    pres = pt_optional_get_ipaddr(pt, "connection.udpDataIp",
                                  (struct in_addr &)udpDataIpAddr, "");
    if (pres) {
        config.udpDataIpAddr = udpDataIpAddr;

        pres = pt_optional_get_default<uint8_t>(
            pt, "connection.udpDataIpAddrLen", config.udpDataIpAddrLen, 4);

        pres = pt_optional_get_default<uint16_t>(pt, "connection.udpDataPort",
                                                 config.udpDataPort, 10002);
        if (pres) {
            printf("UDP PORT SET TO %d\n", config.udpDataPort);
        }

        pt_optional_get<uint8_t>(pt, "connection.udpControlOutIpAddrLen",
                                 config.udpControlOutIpAddrLen);

        pt_optional_get<uint16_t>(pt, "connection.udpControlPort",
                                  config.udpControlPort);

        pres =
            pt_optional_get_ipaddr(pt, "connection.udpControlOutIp",
                                   (struct in_addr &)udpControlOutIpAddr, "");
        if (pres) {
            config.udpControlOutIpAddr = udpControlOutIpAddr;
        }
        pres = pt_optional_get_ipaddr(pt, "connection.udpControlInIp",
                                      (struct in_addr &)udpControlInIpAddr, "");
        if (pres) {
            config.udpControlInIpAddr = udpControlInIpAddr;
        }
        pt_optional_get<uint8_t>(pt, "connection.udpControlInIpAddrLen",
                                 config.udpControlInIpAddrLen);

        pt_optional_get<uint16_t>(pt, "connection.udpControlCallbackPort",
                                  config.udpControlCallbackPort);
    }
    bool autoConf = config.udpDataAutoConfig != 0;
    pt_optional_get_default<bool>(pt, "connection.udpDataAutoConfig", autoConf,
                                  autoConf);
    config.udpDataAutoConfig = autoConf;

    pres = pt_optional_get_ipaddr(pt, "connection.tcpDeviceIp",
                                  (struct in_addr &)tcpDeviceIpAddr,
                                  "192.168.0.10");
    config.tcpDeviceIpAddr = tcpDeviceIpAddr;
    if (pres) {
        BOOST_LOG_TRIVIAL(debug)
            << "tcpDeviceIpAddr set to " << (unsigned int)tcpDeviceIpAddr[0]
            << "." << (unsigned int)tcpDeviceIpAddr[1] << "."
            << (unsigned int)tcpDeviceIpAddr[2] << "."
            << (unsigned int)tcpDeviceIpAddr[3];
    } else {
        BOOST_LOG_TRIVIAL(debug) << "tcpDeviceIpAddr not set ";
    }

    // static inline bool pt_optional_get_ipaddr(const
    // boost::property_tree::ptree pt,
    //     const std::string& key, struct in_addr& inaddr, std::string
    //     defaultAddress) ;

    pt_optional_get_default<uint8_t>(pt, "connection.tcpDeviceIpAddrLen",
                                     config.tcpDeviceIpAddrLen, 4);

    pt_optional_get<uint16_t>(pt, "connection.tcpDataPort", config.tcpDataPort);

    pt_optional_get_default<uint16_t>(pt, "connection.tcpControlPort",
                                      config.tcpControlPort, 10001);

    // can't auto-convert int32 to FrameMode, so do it manually:
    try {
        frameMode =
            pt.get<int32_t>("connection.frameMode", BTA_FrameModeDistAmp);
        config.frameMode = (BTA_FrameMode)frameMode;
        BOOST_LOG_TRIVIAL(debug)
            << "BtaWrapper::parseConfig Read frameMode: " << config.frameMode;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug)
            << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }

    // channel selection:
    std::string channels;
    hasChannels =
        pt_optional_get<std::string>(pt, "connection.channels", channels);
    if (hasChannels) {
        cout << "SETTING CHANNELS! " << channels;
        parseChannelSelection(channels);
    }

    pt_optional_get_default<uint16_t>(pt, "connection.frameQueueLength",
                                      config.frameQueueLength, 10);

    bool hasFile = pt_optional_get<std::string>(
        pt, "connection.bltstreamFilename", bltstreamFilename);
    if (hasFile && bltstreamFilename.length()) {
        config.deviceType = BTA_DeviceTypeGenericBltstream;
    }

    pt_optional_get_default<uint8_t>(pt, "connection.verbosity",
                                     config.verbosity, 5);

    pt_optional_get_default<uint16_t>(pt, "connection.frameQueueLength",
                                      config.frameQueueLength, 10);

    int32_t i32;
    pt_optional_get<int32_t>(pt, "connection.frameQueueMode", i32);
    config.frameQueueMode = (BTA_QueueMode)i32;

    pt_optional_get<int32_t>(pt, "connection.deviceType", i32);
    config.deviceType = (BTA_DeviceType)i32;

    return 0;
}

/*int BtaWrapper::setConfig(BTA_Config inConfig) {
    BTAinitConfig(&config);
    memcpy(config,inConfig,sizeof(BTA_Config));

}*/

int BtaWrapper::reConnect()
{
    handle = 0;
    if (connect()) return 1;
    return 0;
}

int BtaWrapper::connect()
{
    if (isConnected()) {
        BOOST_LOG_TRIVIAL(warning) << "The camera is already connected.";
        return -1;
    }
    if (state == connecting) {
        BOOST_LOG_TRIVIAL(info) << "The camera is already connecting.";
        return -1;
    }
    state = connecting;

    config.infoEventEx2 = infoEventCbEx2;
    config.frameArrivedEx2 = &frameArrivedEx2;
    config.userArg = this;

    if (calibFileName.length()) {
        config.calibFileName = (uint8_t *)calibFileName.c_str();
    } else {
        config.calibFileName = NULL;
    }

    if (bltstreamFilename.length()) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::connect() - enabling playback";
        config.bltstreamFilename = (uint8_t *)bltstreamFilename.c_str();
        config.deviceType = BTA_DeviceTypeBltstream;
        async = true;
    } else {
        config.bltstreamFilename = NULL;
        async = true;
    }

    BOOST_LOG_TRIVIAL(info) << "BtaWrapper::connect() Read tcpDeviceIpAddr: "
                            << (int)config.tcpDeviceIpAddr[0] << "."
                            << (int)config.tcpDeviceIpAddr[1] << "."
                            << (int)config.tcpDeviceIpAddr[2] << "."
                            << (int)config.tcpDeviceIpAddr[3];

    if (config.udpDataIpAddr) {
        BOOST_LOG_TRIVIAL(info) << "BtaWrapper::connect() Read udpDataIpAddr: "
                                << (int)config.udpDataIpAddr[0] << "."
                                << (int)config.udpDataIpAddr[1] << "."
                                << (int)config.udpDataIpAddr[2] << "."
                                << (int)config.udpDataIpAddr[3] << ".";
    }
    BOOST_LOG_TRIVIAL(info)
        << "BtaWrapper::connect() shm " << (int)config.shmDataEnabled;
    // hack on:
    // config.shmDataEnabled = 1;

    status = BTAopen(&config, &handle);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "BtaWrapper::connect() BTAopen: Could "
                                      "not connect to the camera. status: "
                                   << status;
        state = disconnected;
        return status;
    }
    BOOST_LOG_TRIVIAL(debug)
        << "BtaWrapper::connect() Camera connected sucessfully. status: "
        << status;
    state = connected;

    if (hasChannels) {
        status = setChannels();
        if (status < 0) {
            BOOST_LOG_TRIVIAL(warning)
                << "BtaWrapper::connect() setChannels() failed with: "
                << status;
            state = error;
        } else {
            BOOST_LOG_TRIVIAL(debug)
                << "BtaWrapper::connect() setChannels() status: " << status;
        }
    }

    status = BTAgetDeviceInfo(handle, &deviceInfo);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "Could not get device info. " << status;
        state = error;
        return -1;
    }
    BOOST_LOG_TRIVIAL(debug)
        << "Retrieved device info: \n"
        << "deviceType: " << hex << deviceInfo->deviceType << dec << "\n"
        << "serialNumber: " << deviceInfo->serialNumber << "\n"
        << "firmware version " << deviceInfo->firmwareVersionMajor << "."
        << deviceInfo->firmwareVersionMinor << "."
        << deviceInfo->firmwareVersionNonFunc << std::endl;

    device = deviceInfo->deviceType;
    BOOST_LOG_TRIVIAL(debug)
        << "Service running: " << (int)BTAisRunning(handle);
    BOOST_LOG_TRIVIAL(debug)
        << "Connection up: " << (int)BTAisConnected(handle);

    if (bltstreamFilename.length()) {
        BOOST_LOG_TRIVIAL(debug)
            << "BtaWrapper::connect() - auto playback speed";
        BTAsetLibParam(handle, BTA_LibParamStreamAutoPlaybackSpeed, 1);
        status = BTAsetFrameRate(handle, 2.0f);
    }
    if (false) {                   // set AEXP, AWB for color sensor 0
        uint32_t regs[] = {0x03};  //
        uint32_t nregs = sizeof(regs);
        status = BTAwriteRegister(handle, 0x0e1, regs, &nregs);
        // handle, uint32_t address, uint32_t *data, uint32_t *registerCount
    }
    if (false) {                        // set flying pixel filter
        uint32_t regs[] = {0x1 << 13};  //
        uint32_t nregs = sizeof(regs);
        status = BTAwriteRegister(handle, 0x0e1, regs, &nregs);
        // handle, uint32_t address, uint32_t *data, uint32_t *registerCount
    }

    state = connected;
    return 0;
}

bool BtaWrapper::parseChannelSelection(const std::string &chans)
{
    vector<string> strings;
    istringstream f(chans);

    theChannels = chans;

    string s;
    while (getline(f, s, ',')) {
        cout << "[" << s << "]" << endl;
        strings.push_back(s);
    }
    numChannels = strings.size();

    cout << strings.size() << " GET channels" << endl;
    BTA_Status ret;

    for (size_t i = 0; i < strings.size(); i++) {
        chanSelectionName[i] = strings[i];
        if ("x" == strings[i]) {
            channels[i] = BTA_ChannelSelectionX;
        } else if ("y" == strings[i]) {
            channels[i] = BTA_ChannelSelectionY;
        } else if ("z" == strings[i]) {
            channels[i] = BTA_ChannelSelectionZ;
        } else if ("distance" == strings[i]) {
            channels[i] = BTA_ChannelSelectionDistance;
        } else if ("amplitude" == strings[i]) {
            channels[i] = BTA_ChannelSelectionAmplitude;
        } else if ("confidence" == strings[i]) {
            channels[i] = BTA_ChannelSelectionConfidence;
        } else if ("color0" == strings[i]) {
            channels[i] = BTA_ChannelSelectionColor0;
        } else if ("overlay0" == strings[i]) {
            channels[i] = BTA_ChannelSelectionOverlay0;
        } else if ("color1" == strings[i]) {
            channels[i] = BTA_ChannelSelectionColor1;
        } else if ("overlay1" == strings[i]) {
            channels[i] = BTA_ChannelSelectionOverlay1;
        } else if ("amplitude8" == strings[i]) {
            channels[i] = BTA_ChannelSelectionAmplitude8;
        } else {
            cout << " WARN! unknown channel " << strings[i] << endl;
            chanSelectionName[i] = "invalid";
            ret = BTA_StatusInvalidParameter;
        }
    }
    return ret == BTA_StatusOk;
}

BTA_Status BtaWrapper::setChannels()
{
    BTA_Status ret;
    if (!hasChannels) {
        return BTA_StatusIllegalOperation;
    }
    cout << " SET CHANS to " << numChannels << endl;
    ret = BTAsetChannelSelection(handle, channels, numChannels);
    return ret;
}

int BtaWrapper::disconnect()
{
    if (deviceInfo != NULL) {
        status = BTAfreeDeviceInfo(deviceInfo);
        if (status != BTA_StatusOk) {
            BOOST_LOG_TRIVIAL(warning)
                << "Could not free device info. status: " << status;
        }
        deviceInfo = NULL;
    }
    cout << "BtaWrapper::disconnect() isConnected: " << isConnected() << endl;
    if (isConnected()) {
        status = BTAclose(&handle);
        if (status != BTA_StatusOk) {
            BOOST_LOG_TRIVIAL(warning)
                << "Could not disconnect. Status: " << status;
        }
        // Setting the handle to 0 as the bta_p100 keeps returning connected
        handle = 0;
    }
    state = disconnected;

    return 0;
}

bool BtaWrapper::isConnected() const
{
    // cout << "BTAisConnected(handle): " << (int)BTAisConnected(handle) <<
    // endl;
    return (state == connecting || state == connected ||
            BTAisConnected(handle));
}

int BtaWrapper::capture(char *&buffer)
{
    BTA_Frame *frame;
    int i;
    BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::capture() async? " << async;
    return 0;
    for (i = 0; i <= retries; i++) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::capture().BTAgetFrame";
        status = BTAgetFrame(handle, &frame, 1000);
        if (status != BTA_StatusOk) {
            if (status ==
                BTA_StatusTimeOut) {  // no data available yet - sleep a little
                sleep(2);
                continue;
            }
            BOOST_LOG_TRIVIAL(warning)
                << "Could no capture frame. status: " << status;
            continue;
        } else {
            break;
        }
    }
    if (i > retries) {
        BOOST_LOG_TRIVIAL(warning) << "Could not capture.";
        return 0;
    }

    // BOOST_LOG_TRIVIAL(debug) << "Frame captured";
    buffer = (char *)frame;
    /*
    BOOST_LOG_TRIVIAL(debug) << "frame size: " << sizeof(BTA_Frame);
    BOOST_LOG_TRIVIAL(debug) << "BTA_CHANNEL size: " << sizeof(BTA_Channel);
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionNonFunc: " <<
    (int)frame->firmwareVersionNonFunc; BOOST_LOG_TRIVIAL(debug) <<
    "frame->firmwareVersionMinor: " << (int)frame->firmwareVersionMinor;
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionMajor: " <<
    (int)frame->firmwareVersionMajor; BOOST_LOG_TRIVIAL(debug) <<
    "frame->mainTemp: " << (float)frame->mainTemp; BOOST_LOG_TRIVIAL(debug) <<
    "frame->ledTemp " << (float)frame->ledTemp; BOOST_LOG_TRIVIAL(debug) <<
    "frame->genericTemp " << (float)frame->genericTemp; BOOST_LOG_TRIVIAL(debug)
    << "frame->frameCounter " << (float)frame->frameCounter;
    BOOST_LOG_TRIVIAL(debug) << "frame->timeStamp " << frame->timeStamp;
    BOOST_LOG_TRIVIAL(debug) << "frame->channelsLen: " <<
    (int)frame->channelsLen;
    //buffer = (char *)malloc(sizeof(BTA_Frame));
    //memcpy(buffer,frame,sizeof(BTA_Frame));
    */
    return 1;
}

uint32_t BtaWrapper::readRegister(unsigned int reg)
{
    uint32_t usValue;
    status = BTAreadRegister(handle, reg, &usValue, 0);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "Could read reg: status" << status;
        return status;
    }
    return usValue;
}

int BtaWrapper::writeRegister(uint32_t reg, uint32_t data)
{
    status = BTAwriteRegister(handle, reg, &data, 0);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "Could write reg. status: " << status;
        return -1;
    }
    BOOST_LOG_TRIVIAL(debug) << "Register: " << reg << " set to: " << data;
    return 0;
}

char *BtaWrapper::loadFrame(char *data, std::string /*ext*/)
{
    BTA_Frame *frame = (BTA_Frame *)malloc(sizeof(BTA_Frame));
    memcpy(frame, data, sizeof(BTA_Frame));

#ifdef CM_DEBUG
    namespace logging = boost::log;
    logging::core::get()->set_filter(logging::trivial::severity >=
                                     logging::trivial::debug);
#endif

    BOOST_LOG_TRIVIAL(debug) << "frame size: " << sizeof(BTA_Frame);
    BOOST_LOG_TRIVIAL(debug) << "BTA_CHANNEL size: " << sizeof(BTA_Channel);
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionNonFunc: "
                             << (int)frame->firmwareVersionNonFunc;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->firmwareVersionMinor: " << (int)frame->firmwareVersionMinor;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->firmwareVersionMajor: " << (int)frame->firmwareVersionMajor;
    BOOST_LOG_TRIVIAL(debug) << "frame->mainTemp: " << (float)frame->mainTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->ledTemp " << (float)frame->ledTemp;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->genericTemp " << (float)frame->genericTemp;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->frameCounter " << (float)frame->frameCounter;
    BOOST_LOG_TRIVIAL(debug) << "frame->timeStamp " << (int)frame->timeStamp;
    cout << endl;

    // if (UNIX && ext == ".rw")
    memcpy(&frame->channelsLen, data + 28, 1);

    BOOST_LOG_TRIVIAL(debug)
        << "frame->channelsLen: " << (int)frame->channelsLen;
    cout << endl;

    // TODO channelsLen is nº of channels or the byte lenght??
    frame->channels =
        (BTA_Channel **)malloc((sizeof(BTA_Channel *) * frame->channelsLen));

    // TODO channelsLen is nº of channels or the byte lenght??
    int pos = sizeof(BTA_Frame);
    // if (UNIX && ext == ".rw")
    pos = 32;
    for (int i = 0; i < frame->channelsLen; i++) {
        // BTA_Channel *test = ()(frame+pos)
        // TEST

        // frame->channels[i] = (BTA_Channel *)malloc(28);
        // memcpy(frame->channels[i],data+pos,28);

        frame->channels[i] = (BTA_Channel *)malloc(sizeof(BTA_Channel));
        memcpy(frame->channels[i], data + pos, sizeof(BTA_Channel));
        // memcpy(frame->channels[i],data+pos,28);

        BOOST_LOG_TRIVIAL(debug) << "frame size: " << sizeof(BTA_Frame);
        BOOST_LOG_TRIVIAL(debug) << "BTA_CHANNEL size: " << sizeof(BTA_Channel);
        BOOST_LOG_TRIVIAL(debug)
            << "frame->channels[i]->id: " << (int)frame->channels[i]->id;
        BOOST_LOG_TRIVIAL(debug)
            << "frame->channels[i]->xRes: " << (int)frame->channels[i]->xRes;
        BOOST_LOG_TRIVIAL(debug)
            << "frame->channels[i]->yRes: " << (int)frame->channels[i]->yRes;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->dataFormat: "
                                 << (int)frame->channels[i]->dataFormat;
        BOOST_LOG_TRIVIAL(debug)
            << "frame->channels[i]->unit: " << (int)frame->channels[i]->unit;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->integrationTime: "
                                 << (int)frame->channels[i]->integrationTime;
        BOOST_LOG_TRIVIAL(debug)
            << "frame->channels[i]->modulationFrequency: "
            << (int)frame->channels[i]->modulationFrequency;
        cout << endl;

        // TEST
        // pos+=28;
        pos += sizeof(BTA_Channel);
        // if (UNIX && ext == ".rw")
        // pos -= ( sizeof(BTA_Channel) - 28 ); /Changed for arm, change if it
        // fails for windows files
        pos -= (sizeof(BTA_Channel) - 32);
        int dataSize = sizeof(float);
        if (frame->channels[i]->dataFormat == BTA_DataFormatUInt16)
            dataSize = sizeof(unsigned short);
        if (frame->channels[i]->dataFormat == BTA_DataFormatUInt32)
            dataSize = sizeof(unsigned int);

        BOOST_LOG_TRIVIAL(debug)
            << (int)(dataSize *
                     (frame->channels[i]->xRes * frame->channels[i]->yRes));
        dataSize *= frame->channels[i]->xRes * frame->channels[i]->yRes;

        frame->channels[i]->data = (uint8_t *)malloc(dataSize);
        memcpy(frame->channels[i]->data, data + pos, dataSize);

        pos += dataSize;
    }
    BOOST_LOG_TRIVIAL(debug) << "frame loaded.";
    return (char *)frame;
}

char *BtaWrapper::serializeFrame(char *data, size_t &size)
{
    BTA_Frame *frame = (BTA_Frame *)data;
    vector<size_t> channels(frame->channelsLen);
    // size_t channels[frame->channelsLen];
    // channelsLen is nº of channels
    size = sizeof(BTA_Frame) + sizeof(BTA_Channel) * frame->channelsLen;

    BOOST_LOG_TRIVIAL(info) << "frame size: " << sizeof(BTA_Frame);
    BOOST_LOG_TRIVIAL(info) << "BTA_CHANNEL size: " << sizeof(BTA_Channel);
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionNonFunc "
                             << (int)frame->firmwareVersionNonFunc;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->firmwareVersionMinor " << (int)frame->firmwareVersionMinor;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->firmwareVersionMajor " << (int)frame->firmwareVersionMajor;
    BOOST_LOG_TRIVIAL(debug) << "frame->mainTemp " << (float)frame->mainTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->ledTemp " << (float)frame->ledTemp;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->genericTemp " << (float)frame->genericTemp;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->frameCounter " << (int)frame->frameCounter;
    BOOST_LOG_TRIVIAL(debug) << "frame->timeStamp " << (int)frame->timeStamp;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->channelsLen: " << (int)frame->channelsLen;
    BOOST_LOG_TRIVIAL(debug)
        << "frame->sequenceCounter: " << (int)frame->sequenceCounter;

    for (int i = 0; i < frame->channelsLen; i++) {
        int dataSize = sizeof(float);
        if (frame->channels[i]->dataFormat == BTA_DataFormatUInt16)
            dataSize = sizeof(unsigned short);
        else if (frame->channels[i]->dataFormat == BTA_DataFormatUInt32)
            dataSize = sizeof(unsigned int);
        else if (frame->channels[i]->dataFormat == BTA_DataFormatFloat32)
            dataSize = sizeof(unsigned int);
        else {
            BOOST_LOG_TRIVIAL(error)
                << "unknown dataFormat: 0x" << hex
                << (int)(frame->channels[i]->dataFormat) << dec;
        }

        channels[i] =
            frame->channels[i]->xRes * frame->channels[i]->yRes * dataSize;
        size += channels[i];
    }

    char *raw = (char *)malloc(size);
    size_t count = sizeof(BTA_Frame);
    memcpy(raw, frame, sizeof(BTA_Frame));

    for (int i = 0; i < frame->channelsLen; i++) {
        BOOST_LOG_TRIVIAL(debug)
            << "frame->channels[i]->id: " << (int)frame->channels[i]->id;
        BOOST_LOG_TRIVIAL(debug)
            << "frame->channels[i]->xRes: " << (int)frame->channels[i]->xRes;
        BOOST_LOG_TRIVIAL(debug)
            << "frame->channels[i]->yRes: " << (int)frame->channels[i]->yRes;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->dataFormat: "
                                 << (int)frame->channels[i]->dataFormat;
        BOOST_LOG_TRIVIAL(debug)
            << "frame->channels[i]->unit: " << (int)frame->channels[i]->unit;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->integrationTime: "
                                 << frame->channels[i]->integrationTime;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->modulationFrequency: "
                                 << frame->channels[i]->modulationFrequency;

        BOOST_LOG_TRIVIAL(debug)
            << "sizeof(BTA_Channel): " << sizeof(BTA_Channel);
        BOOST_LOG_TRIVIAL(debug) << "channels[i]: " << (int)channels[i];

        memcpy(raw + count, frame->channels[i], sizeof(BTA_Channel));

        count += sizeof(BTA_Channel);

        size_t dataSize = channels[i];

        memcpy(raw + count, frame->channels[i]->data, dataSize);

        count += dataSize;
    }
    return raw;
}

int BtaWrapper::freeFrame(char *data)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BTA_Frame *frame = (BTA_Frame *)data;
    status = BTAfreeFrame(&frame);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "Could not connect to the camera.";
        return -1;
    }
    frame = NULL;
    data = NULL;
    return 0;
}

int BtaWrapper::getDistances(float *&depth, int &size, char *data)
{
    BTA_Frame *frame = (BTA_Frame *)data;

    void *disVoid;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    uint16_t xRes, yRes;
    // BOOST_LOG_TRIVIAL(debug) << "BTAgetDistances()";
    status = BTAgetDistances(frame, &disVoid, &dataFormat, &unit, &xRes, &yRes);

    if (status != BTA_StatusOk) {
        return -1;
    }

    if (size != xRes * yRes) {
        size = xRes * yRes;
        BOOST_LOG_TRIVIAL(debug) << "Size does not match! " << size;
        BOOST_LOG_TRIVIAL(debug) << "xRes: " << xRes;
        BOOST_LOG_TRIVIAL(debug) << "yRes: " << yRes;

        return -1;
    }

    if (dataFormat == BTA_DataFormatUInt16) {
        // BOOST_LOG_TRIVIAL(debug) << "unit: " << unit;
        if (unit == BTA_UnitMillimeter) {
            unsigned short *distances = (unsigned short *)disVoid;

            for (int j = 0; j < size; j++) {
                if (distances[j] <= 0x0001 || distances[j] > 0xFA00)
                    depth[j] = 0.0;
                // depth[j] = std::numeric_limits<float>::quiet_NaN();
                else
                    depth[j] = (distances[j] / 1000.f);
            }
            return 1;
        }
    } else if (dataFormat == BTA_DataFormatFloat32) {
        if (unit == BTA_UnitMeter) {
            float *distances = (float *)disVoid;

            for (int j = 0; j < size; j++) {
                if (distances[j] <= 0.1f || distances[j] > 20.0f)
                    depth[j] = std::numeric_limits<float>::quiet_NaN();
                else
                    depth[j] = distances[j];
                // BOOST_LOG_TRIVIAL(warning) << depth[j];
            }
            return 1;
        }
    } else {
        BOOST_LOG_TRIVIAL(warning) << "Unknown data format! " << dataFormat;
    }

    return -1;
}

int BtaWrapper::getAmplitudes(unsigned short *&amplitudes, int &size,
                              char *data)
{
    BTA_Frame *frame = (BTA_Frame *)data;
    void *amplVoid;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    uint16_t xRes, yRes;
    int sz = size;
    status =
        BTAgetAmplitudes(frame, &amplVoid, &dataFormat, &unit, &xRes, &yRes);

    if (status != BTA_StatusOk) {
        return -1;
    }

    if (sz != xRes * yRes) {
        size = xRes * yRes;
        BOOST_LOG_TRIVIAL(warning)
            << "Size does not match! " << size << " " << xRes << " " << yRes
            << " " << (xRes * yRes) << endl;
        BOOST_LOG_TRIVIAL(debug) << "xRes: " << xRes;
        BOOST_LOG_TRIVIAL(debug) << "yRes: " << yRes;

        return -1;
    }

    if (dataFormat == BTA_DataFormatUInt16) {
        if (unit == BTA_UnitUnitLess) {
            size = xRes * yRes;

            memcpy(amplitudes, amplVoid, sizeof(unsigned short) * size);

            return 1;
        }
    } else if (dataFormat == BTA_DataFormatFloat32) {
        // Keeping BTA_UnitMeter because of error in BtaApi v1.0.0
        if (unit == BTA_UnitUnitLess || unit == BTA_UnitMeter) {
            float *amp = (float *)amplVoid;
            // printf("Got amplitude data\n");
            size = xRes * yRes;

            for (int j = 0; j < size; j++) {
                amplitudes[j] = (unsigned short)amp[j];
            }
            return 1;
        }
    } else {
        BOOST_LOG_TRIVIAL(warning) << "Unknown data format! " << dataFormat;
    }

    return -1;
}

int BtaWrapper::reset() { return BTAsendReset(handle); }

int BtaWrapper::getDisSize(char *data, int &x, int &y)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BTA_Frame *frame = (BTA_Frame *)data;
    uint16_t *distances;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    // BOOST_LOG_TRIVIAL(debug) << "BTAgetDistances()";
    status = BTAgetDistances(frame, (void **)&distances, &dataFormat, &unit,
                             (uint16_t *)&x, (uint16_t *)&y);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status == BTA_StatusOk) {
        return 1;
    }
    return -1;
}

int BtaWrapper::getAmpSize(char *data, int &x, int &y)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BTA_Frame *frame = (BTA_Frame *)data;

    BTA_DataFormat dataFormat;
    uint16_t *amplitudes;
    BTA_Unit unit;
    // BOOST_LOG_TRIVIAL(debug) << "BTAgetAmplitudes()";
    status = BTAgetAmplitudes(frame, (void **)&amplitudes, &dataFormat, &unit,
                              (uint16_t *)&x, (uint16_t *)&y);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status == BTA_StatusOk) {
        return 1;
    }
    return -1;
}

int BtaWrapper::getFrameTime(char *data, unsigned int &timeStamp)
{
    BTA_Frame *frame = (BTA_Frame *)data;
    if (frame != NULL) {
        timeStamp = frame->timeStamp;
        return 1;
    }
    return -1;
}

int BtaWrapper::getFrameCounter(char *data, unsigned int &counter)
{
    BTA_Frame *frame = (BTA_Frame *)data;
    if (frame != NULL) {
        counter = frame->frameCounter;
        return 1;
    }
    return -1;
}

int BtaWrapper::getFrameRef(char *data, unsigned int &mf, unsigned int &it)
{
    BTA_Frame *frame = (BTA_Frame *)data;
    if (frame != NULL && frame->channelsLen > 0) {
        BTA_Channel *ch = frame->channels[0];
        mf = ch->modulationFrequency;
        it = ch->integrationTime;
        return 1;
    }
    return -1;
}

float mainTemp;  //<<< Main-board/processor temperature sensor in degree Celcius
float ledTemp;   //<<< Led-board temperature sensor in degree Celcius
float genericTemp;

int BtaWrapper::getTemps(char *data, float &mt, float &lt, float &gt)
{
    BTA_Frame *frame = (BTA_Frame *)data;
    mt = frame->mainTemp;
    lt = frame->ledTemp;
    gt = frame->genericTemp;
    return 1;
}

int BtaWrapper::getMainTemp(char *data, float &mt)
{
    BTA_Frame *frame = (BTA_Frame *)data;
    mt = frame->mainTemp;
    return 1;
}
int BtaWrapper::getLedTemp(char *data, float &lt)
{
    BTA_Frame *frame = (BTA_Frame *)data;
    lt = frame->ledTemp;
    return 1;
}
int BtaWrapper::getGenericTemp(char *data, float &gt)
{
    BTA_Frame *frame = (BTA_Frame *)data;
    gt = frame->genericTemp;
    return 1;
}

float BtaWrapper::getGlobalOffset()
{
    float val;
    status = BTAgetGlobalOffset(handle, &val);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "getGlobalOffset() Status: " << status;
        return -1;
    }
    return val;
}

int BtaWrapper::setGlobalOffset(float val)
{
    status = BTAsetGlobalOffset(handle, val);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "setGlobalOffset() Status: " << status;
        return -1;
    }
    return 1;
}

unsigned int BtaWrapper::getIntegrationTime()
{
    unsigned int it;
    status = BTAgetIntegrationTime(handle, &it);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return it;
}

float BtaWrapper::getFrameRate()
{
    float fr;
    status = BTAgetFrameRate(handle, &fr);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return fr;
}

unsigned long BtaWrapper::getModulationFrequency()
{
    unsigned int it;
    status = BTAgetModulationFrequency(handle, &it);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return it;
}

int BtaWrapper::setIntegrationTime(unsigned int it)
{
    status = BTAsetIntegrationTime(handle, it);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return 1;
}

int BtaWrapper::setFrameRate(float fr)
{
    status = BTAsetFrameRate(handle, fr);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return 1;
}

int BtaWrapper::setModulationFrequency(unsigned long mf)
{
    status = BTAsetModulationFrequency(handle, mf);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return 1;
}

int BtaWrapper::startGrabbing(std::string filename)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BTA_GrabbingConfig grabbingConfig;
    status = BTAinitGrabbingConfig(&grabbingConfig);
    errorHandling(status);
    // TODO check filename; renaming; etc..
    grabbingConfig.filename = (unsigned char *)filename.c_str();
    status = BTAstartGrabbing(handle, &grabbingConfig);
    errorHandling(status);
    // TODO check status;
    return 1;
}

int BtaWrapper::stopGrabbing()
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    status = BTAstartGrabbing(handle, 0);
    errorHandling(status);
    return 1;
}

int BtaWrapper::saveRaw(string fileName, char *data)
{
    size_t data_serialized_size;
    char *data_serialized = serializeFrame(data, data_serialized_size);

    char *raw = new char[sizeof(FrameHeader) + data_serialized_size];

    FrameHeader header;
    header.manufacturer = manufacturer;
    // BOOST_LOG_TRIVIAL(debug) << "camera->getDevice() " <<
    // camera->getDevice();
    header.device = device;
    // BOOST_LOG_TRIVIAL(debug) << "header.device " << header.device;
    header.lenght = static_cast<int>(data_serialized_size);

    memcpy(raw, &header, sizeof(FrameHeader));
    memcpy(raw + sizeof(FrameHeader), data_serialized, data_serialized_size);
    free(data_serialized);
    BOOST_LOG_TRIVIAL(debug)
        << "Saving file: " << fileName
        << ", size: " << data_serialized_size + sizeof(FrameHeader);
    // Frame *test = (Frame *)raw;
    // BOOST_LOG_TRIVIAL(debug) << "Tests frame: " << test->device;
    ofstream f;
    f.open(fileName.c_str(), ios::out | ios_base::binary);
    f.write(raw, sizeof(FrameHeader) + data_serialized_size);
    f.close();
    BOOST_LOG_TRIVIAL(debug) << "Saved file: " << fileName;
    delete[] raw;

    return 0;
}

char *BtaWrapper::loadRaw(string rawFile)
{
    FILE *raw = fopen(rawFile.c_str(), "rb");
    if (raw == NULL) {
        BOOST_LOG_TRIVIAL(warning) << "Could not open file: " << rawFile;
        return NULL;
    }

    FrameHeader header;

    size_t ret;
    ret = fread(&header, sizeof(FrameHeader), 1, raw);
    if (ret == 0) {
        BOOST_LOG_TRIVIAL(warning) << "Could not read file: " << rawFile;
        fclose(raw);
        return NULL;
    }

    /*if (checkRaw(header)){
        BOOST_LOG_TRIVIAL(warning) << "Raw frame does not match sensor device.";
        return NULL;
    }
    BOOST_LOG_TRIVIAL(debug) << "sizeof(FrameHeader): " <<
    sizeof(FrameHeader);*/

    char *frame_data = new char[header.lenght];
    ret = fread(frame_data, sizeof(char), header.lenght, raw);
    if (ret == 0) {
        BOOST_LOG_TRIVIAL(warning) << "Could not read file: " << rawFile;
        fclose(raw);
        delete[] frame_data;
        return NULL;
    }

    BOOST_LOG_TRIVIAL(debug) << "Raw frame read.";
    fclose(raw);

    string ext = rawFile.substr(rawFile.find_last_of("."),
                                rawFile.length() - rawFile.find_last_of("."));

    char *retu = loadFrame(frame_data, ext);
    delete[] frame_data;
    return retu;
}

BTA_DeviceType BtaWrapper::getDeviceType() const { return config.deviceType; }

void BtaWrapper::setDeviceType(const BTA_DeviceType &value)
{
    config.deviceType = value;
}

int BtaWrapper::getLibParam(int &param, float &data)
{
    return BTAgetLibParam(handle, (BTA_LibParam)param, &data);
}

int BtaWrapper::setLibParam(int &param, float &data)
{
    return BTAsetLibParam(handle, (BTA_LibParam)param, data);
}

using namespace std;

static void freeMetaData(BTA_Channel *chan)
{
    for (uint32_t i = 0; i < chan->metadataLen; i++) {
        delete chan->metadata[i];
    }
    delete chan->metadata;
}

static void freeChannel(BTA_Channel *chan)
{
    freeMetaData(chan);
    delete chan->metadata;
    delete chan->data;
    delete chan;
}

static void freeChannels(BTA_Frame *frame)
{
    for (uint32_t i = 0; i < frame->channelsLen; i++) {
        freeChannel(frame->channels[i]);
    }
    delete frame->channels;
}

static inline void cpyMetaData(BTA_Channel *dst, const BTA_Channel *src)
{
    for (uint32_t i = 0; i < src->metadataLen; i++) {
        BTA_Metadata *s = src->metadata[i];
        BTA_Metadata *d = dst->metadata[i];
        if (d->dataLen != s->dataLen) {
            free(d->data);
            d->data = malloc(s->dataLen);
            d->dataLen = s->dataLen;
        }
        memcpy(d->data, s->data, d->dataLen);
    }
}
static bool debugCvt = false;

std::string getChannelSelectionName(BTA_ChannelSelection sel)
{
    switch (sel) {
        case BTA_ChannelSelectionInactive:
            return "inactive";
        case BTA_ChannelSelectionDistance:
            return "distance";
        case BTA_ChannelSelectionAmplitude:
            return "amplitude";
        case BTA_ChannelSelectionX:
            return "x";
        case BTA_ChannelSelectionY:
            return "y";
        case BTA_ChannelSelectionZ:
            return "z";
        case BTA_ChannelSelectionConfidence:
            return "confidence";
        case BTA_ChannelSelectionHeightMap:
            return "heightmap";
        case BTA_ChannelSelectionStdev:
            return "stdev";
        case BTA_ChannelSelectionColor0:
            return "color0";
        case BTA_ChannelSelectionOverlay0:
            return "overlay0";
        case BTA_ChannelSelectionColor1:
            return "color1";
        case BTA_ChannelSelectionOverlay1:
            return "overlay1";
        case BTA_ChannelSelectionAmplitude8:
            return "amplitude8";
        default:
            return "unknown";
    }
}

std::string getChannelTypeName(BTA_ChannelId cid)
{
    switch (cid) {
        case BTA_ChannelIdUnknown:
            return "UNKNOWN";
        case BTA_ChannelIdDistance:
            return "distance";
        case BTA_ChannelIdAmplitude:
            return "amplitude";
        case BTA_ChannelIdX:
            return "x";
        case BTA_ChannelIdY:
            return "y";
        case BTA_ChannelIdZ:
            return "z";
        case BTA_ChannelIdConfidence:
            return "confidence";
        case BTA_ChannelIdFlags:
            return "flags";
        case BTA_ChannelIdPhase0:
            return "phase0";
        case BTA_ChannelIdPhase90:
            return "phase90";
        case BTA_ChannelIdPhase180:
            return "phase180";
        case BTA_ChannelIdPhase270:
            return "phase270";
        case BTA_ChannelIdRawPhase:
            return "rawPhase";
        case BTA_ChannelIdRawQ:
            return "rawQ";
        case BTA_ChannelIdRawI:
            return "rawI";
        case BTA_ChannelIdTest:
            return "test";
        case BTA_ChannelIdColor:
            return "color";
        // case BTA_ChannelIdGrayScale:
        //     return "gray";
        case BTA_ChannelIdBalance:
            return "balance";
        case BTA_ChannelIdCustom01:
            return "custom01";
        case BTA_ChannelIdCustom02:
            return "custom02";
        case BTA_ChannelIdCustom03:
            return "custom03";
        case BTA_ChannelIdCustom04:
            return "custom04";
        case BTA_ChannelIdCustom05:
            return "custom05";
        case BTA_ChannelIdCustom06:
            return "custom06";
        case BTA_ChannelIdCustom07:
            return "custom07";
        case BTA_ChannelIdCustom08:
            return "custom08";
        case BTA_ChannelIdCustom09:
            return "custom09";
        case BTA_ChannelIdCustom10:
            return "custom10";

        default:
            return "UNDEFINED CID!";
    }
}

static void cpyChannel(BTA_Channel *dst, const BTA_Channel *src)
{
    dst->id = src->id;
    dst->xRes = src->xRes;
    dst->yRes = src->yRes;
    dst->dataFormat = src->dataFormat;
    dst->unit = src->unit;
    dst->integrationTime = src->integrationTime;
    dst->modulationFrequency = src->modulationFrequency;
    dst->lensIndex = src->lensIndex;
    dst->flags = src->flags;
    dst->sequenceCounter = src->sequenceCounter;
    dst->gain = src->gain;

    // copy data
    if (src->dataLen != dst->dataLen) {
        delete[] dst->data;
        dst->data = new uint8_t[src->dataLen];
        dst->dataLen = src->dataLen;
    }
    memcpy(dst->data, src->data, dst->dataLen);

    // copy metadata
    if (src->metadataLen != dst->metadataLen) {
        freeMetaData(dst);
        dst->metadata = new BTA_Metadata *[src->metadataLen];
        for (uint32_t i = 0; i < dst->metadataLen; i++) {
            dst->metadata[i] = new BTA_Metadata;
        }
    }
    cpyMetaData(dst, src);
    if (debugCvt)
        cout << "cpyChan sq " << dst->sequenceCounter << " it "
             << dst->integrationTime << " mf " << dst->modulationFrequency
             << " unit " << dst->unit << " df " << dst->dataFormat << "\t\t"
             << getChannelTypeName(dst->id) << endl;
}

static void cpyChannels(BTA_Frame *dst, const BTA_Frame *src)
{
    if (debugCvt)
        cout << "cpyChannels " << (int)dst->channelsLen << "-"
             << (int)src->channelsLen << endl;
    // num channels does not fit - create new channels array with dummies:
    if (dst->channelsLen != src->channelsLen) {
        freeChannels(dst);
        if (debugCvt)
            cout << "cpyChannels realloc" << (int)dst->channelsLen << "-"
                 << (int)src->channelsLen << endl;
        if (debugCvt)
            cout << "cpyChannels realloc " << src->channelsLen << endl;

        dst->channels = new BTA_Channel *[src->channelsLen];
        dst->channelsLen = src->channelsLen;
        for (int i = 0; i < dst->channelsLen; i++) {
            dst->channels[i] = new BTA_Channel();
        }
    }
    // walk over channels, compare contents and copy channel by channel
    for (int i = 0; i < dst->channelsLen; i++) {
        auto s = src->channels[i];
        auto d = dst->channels[i];
        if (!d) {
            d = new BTA_Channel();
            dst->channels[i] = d;
        }
        if (debugCvt) cout << "cpyChannels c " << i << endl;
        cpyChannel(d, s);
    }
}

static void cpyFrame(BTA_Frame *dst, const BTA_Frame *src)
{
    if (debugCvt) cout << "cpyFrame cl " << (int)src->channelsLen << endl;
    if (debugCvt) cout << "cpyFrame cl " << (int)dst->channelsLen << endl;

    dst->firmwareVersionMajor = src->firmwareVersionMajor;
    dst->firmwareVersionMinor = src->firmwareVersionMinor;
    dst->firmwareVersionNonFunc = src->firmwareVersionNonFunc;
    dst->mainTemp = src->mainTemp;
    dst->ledTemp = src->ledTemp;
    dst->genericTemp = src->genericTemp;
    dst->frameCounter = src->frameCounter;
    dst->timeStamp = src->timeStamp;
    dst->sequenceCounter = src->sequenceCounter;

    cpyChannels(dst, src);

    if (debugCvt)
        cout << "cpyFrame sq "
             << (int)dst->sequenceCounter
             //   << " it " << dst->integrationTime
             //   << " mf " << dst->modulationFrequency
             //   << " unit " << dst-<unit;
             << endl;
}

uint8_t ctr = 0;
// queue handling:
void BtaWrapper::updateFrame(BTA_Frame *frame)
{
    {
        boost::lock_guard<boost::mutex> lock{fillFrameMutex};

        // cout << "updateFrame inLock " << frameToFill->frameCounter << endl;

        cpyFrame(frameToFill, frame);
        frameToFill->sequenceCounter = ctr++;
        hasBeenUpdated = true;
    }
    newFrameCond.notify_one();
    // cout << "updateFrame " << hasBeenUpdated << " " <<
    // frameToFill->frameCounter << endl;
}

BTA_Frame *BtaWrapper::flipFrame()
{
    boost::lock_guard<boost::mutex> lock{fillFrameMutex};
    hasBeenUpdated = false;

    BTA_Frame *tmp = frameInUse;
    frameInUse = frameToFill;
    frameToFill = tmp;  // ->ftf == fiu

    //cout << "flip fill " << (int)frameToFill->sequenceCounter << " use "
    //     << (int)frameInUse->sequenceCounter << endl;

    return frameInUse;
}

BTA_Frame *BtaWrapper::waitForNextFrame()
{  // wait for next frame to arrive....
    // boost::lock_guard<boost::mutex> lock{frameMutex};
    boost::unique_lock<boost::mutex> lock(frameMutex);
    // cout << "waitForNextFrame " << hasBeenUpdated << endl;
    while (!hasBeenUpdated) {
        // cout << "waitForNextFrame ..." << hasBeenUpdated << endl;
        newFrameCond.wait(lock);
    }
    // cout << "waitForNextFrame got one!" << endl;
    return flipFrame();
}
