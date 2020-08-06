#include <iostream>
#include <cmath>
#include <limits>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>


#include <toffy/bta/BtaWrapper.hpp>
#include <toffy/bta/FrameHeader.hpp>
#include <bta.h>

using namespace std;


static void BTA_CALLCONV infoEventCb(BTA_EventId eventId, int8_t *msg) 
{
    //printf("   Callback: infoEvent (%d) %s\n", eventId, msg);
    BOOST_LOG_TRIVIAL(info) << "   Callback: infoEvent (" << eventId << ") " << msg;
    //cout << "   Callback: infoEvent (" << eventId << ") " << msg << endl;
}

static void errorHandling(BTA_Status status) {
    if (status != BTA_StatusOk) {
        char statusString[100];
        BTAstatusToString(status, statusString, strlen(statusString));
        BOOST_LOG_TRIVIAL(warning) << "bta: " << statusString
                                   << ". error id: " << status;
    }
}

std::string BtaWrapper::getBltstream() const
{
    return bltstreamFilename;
}

void BtaWrapper::setBltstream(const std::string &value)
{
    bltstreamFilename = value;
}

BtaWrapper::BtaWrapper()
{

    handle = 0;
    deviceInfo = NULL;
    //TODO is size really needed;
    //setSize(160, 120);
    //TODO create header with manufacturer codes
    manufacturer = 1;
}

BtaWrapper::~BtaWrapper() 
{
    disconnect();
} 

int BtaWrapper::parseConfig(string configFile) {
    boost::property_tree::ptree pt;
    cout << endl << endl << "---------------------------------- " << endl << endl;
    try {
        boost::property_tree::read_xml(configFile, pt);
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(warning) << "Could not open config file: " <<
                                      configFile << ". " << e.what();
        return -1;
    }
    BOOST_LOG_TRIVIAL(debug) << "Config file opened.";
    boost::optional<boost::property_tree::ptree &> opt = pt.get_child_optional("opencv_storage");
    if(opt.is_initialized()) {
        return parseConfig(opt.get());
    } else
        return parseConfig(pt);
}

int BtaWrapper::parseConfig(const boost::property_tree::ptree pt)
{
    BTAinitConfig(&config);
    try {
        //TODO extend to ipv6
        udpDataIpAddr[0] = pt.get<uint8_t>("connection.udpDataIpAddr.n1",224);
        udpDataIpAddr[1] = pt.get<uint8_t>("connection.udpDataIpAddr.n2",0);
        udpDataIpAddr[2] = pt.get<uint8_t>("connection.udpDataIpAddr.n3",0);
        udpDataIpAddr[3] = pt.get<uint8_t>("connection.udpDataIpAddr.n4",1);

        config.udpDataIpAddr = udpDataIpAddr;
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read udpDataIpAddr: "
                                 << (int)pt.get<uint8_t>("connection.udpDataIpAddr.n1",224) << "."
                                 << (int)pt.get<uint8_t>("connection.udpDataIpAddr.n2",0) << "."
                                 << (int)pt.get<uint8_t>("connection.udpDataIpAddr.n3",0) << "."
                                 << (int)pt.get<uint8_t>("connection.udpDataIpAddr.n4",1);
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: "
                                 << e.what();
    }
    try {
        config.udpDataIpAddrLen = pt.get<uint8_t>("connection.udpDataIpAddrLen",4);
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read udpDataIpAddrLen: "
                                 << (int)config.udpDataIpAddrLen ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "Error getting parameters: " << e.what();
    }
    try {
        config.udpDataPort = pt.get<uint16_t>("connection.udpDataPort",10002);
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read udpDataPort: "
                                 << config.udpDataPort ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        udpControlOutIpAddr[0] = pt.get<uint8_t>("connection.udpControlOutIpAddr.n1");
        udpControlOutIpAddr[1] = pt.get<uint8_t>("connection.udpControlOutIpAddr.n2");
        udpControlOutIpAddr[2] = pt.get<uint8_t>("connection.udpControlOutIpAddr.n3");
        udpControlOutIpAddr[3] = pt.get<uint8_t>("connection.udpControlOutIpAddr.n4");
        config.udpControlOutIpAddr = udpControlOutIpAddr;
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read udpDataIpAddr: "
                                 << (int)pt.get<uint8_t>("connection.udpControlOutIpAddr.n1") << "."
                                 << (int)pt.get<uint8_t>("connection.udpControlOutIpAddr.n2") << "."
                                 << (int)pt.get<uint8_t>("connection.udpControlOutIpAddr.n3") << "."
                                 << (int)pt.get<uint8_t>("connection.udpControlOutIpAddr.n4");
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.udpControlOutIpAddrLen = pt.get<uint8_t>("connection.udpControlOutIpAddrLen");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read udpControlOutIpAddrLen: "
                                 << (int)config.udpControlOutIpAddrLen;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.udpControlPort = pt.get<uint16_t>("connection.udpControlPort");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read udpControlPort: "
                                 << config.udpControlPort ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "Error getting parameters: " << e.what();
    }
    try {
        udpControlInIpAddr[0] = pt.get<uint8_t>("connection.udpControlInIpAddr.n1");
        udpControlInIpAddr[1] = pt.get<uint8_t>("connection.udpControlInIpAddr.n2");
        udpControlInIpAddr[2] = pt.get<uint8_t>("connection.udpControlInIpAddr.n3");
        udpControlInIpAddr[3] = pt.get<uint8_t>("connection.udpControlInIpAddr.n4");

        config.udpControlInIpAddr = udpControlInIpAddr;
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read udpDataIpAddr: "
                                 << (int)pt.get<uint8_t>("connection.udpControlInIpAddr.n1") << "."
                                 << (int)pt.get<uint8_t>("connection.udpControlInIpAddr.n2") << "."
                                 << (int)pt.get<uint8_t>("connection.udpControlInIpAddr.n3") << "."
                                 << (int)pt.get<uint8_t>("connection.udpControlInIpAddr.n4");
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.udpControlInIpAddrLen = pt.get<uint8_t>("connection.udpControlInIpAddrLen");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read udpControlInIpAddrLen: "
                                 << (int)config.udpControlInIpAddrLen ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.udpControlCallbackPort = pt.get<uint16_t>("connection.udpControlCallbackPort");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read udpControlCallbackPort: "
                                 << config.udpControlCallbackPort ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        //TODO extend to ipv6
        tcpDeviceIpAddr[0] = pt.get<uint8_t>("connection.tcpDeviceIpAddr.n1",192);
        tcpDeviceIpAddr[1] = pt.get<uint8_t>("connection.tcpDeviceIpAddr.n2",168);
        tcpDeviceIpAddr[2] = pt.get<uint8_t>("connection.tcpDeviceIpAddr.n3",0);
        tcpDeviceIpAddr[3] = pt.get<uint8_t>("connection.tcpDeviceIpAddr.n4",10);

        config.tcpDeviceIpAddr = tcpDeviceIpAddr;
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read tcpDeviceIpAddr: "
                                 << (int)pt.get<uint8_t>("connection.tcpDeviceIpAddr.n1",192) << "."
                                 << (int)pt.get<uint8_t>("connection.tcpDeviceIpAddr.n2",168) << "."
                                 << (int)pt.get<uint8_t>("connection.tcpDeviceIpAddr.n3",0) << "."
                                 << (int)pt.get<uint8_t>("connection.tcpDeviceIpAddr.n4",10);
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.tcpDeviceIpAddrLen = pt.get<uint8_t>("connection.tcpDeviceIpAddrLen",4);
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read tcpDeviceIpAddrLen: "
                                 << (int)config.tcpDeviceIpAddrLen ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.tcpDataPort = pt.get<uint16_t>("connection.tcpDataPort");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read tcpDataPort: "
                                 << config.tcpDataPort ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.tcpControlPort = pt.get<uint16_t>("connection.tcpControlPort",10001);
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read tcpControlPort: "
                                 << config.tcpControlPort ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        uartPortName = pt.get<string>("connection.uartPortName");
        //config.uartPortName = pt.get<int8_t *>("connection.uartPortName");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read uartPortName: "
                                 << uartPortName ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.uartBaudRate = pt.get<uint32_t>("connection.uartBaudRate");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read uartBaudRate: "
                                 << config.uartBaudRate ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.uartDataBits = pt.get<uint8_t>("connection.uartDataBits");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read uartDataBits: "
                                 << config.uartDataBits ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.uartStopBits = pt.get<uint8_t>("connection.uartStopBits");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read uartStopBits: "
                                 << config.uartStopBits ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.uartParity = pt.get<uint8_t>("connection.uartParity");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read uartParity: "
                                 << config.uartParity ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.uartTransmitterAddress = pt.get<uint8_t>("connection.uartTransmitterAddress");
        BOOST_LOG_TRIVIAL(debug) << "Read uartTransmitterAddress: "
                                 << config.uartTransmitterAddress ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.uartReceiverAddress = pt.get<uint8_t>("connection.uartReceiverAddress");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read uartReceiverAddress: "
                                 << config.uartReceiverAddress ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.serialNumber = pt.get<uint32_t>("connection.serialNumber");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read serialNumber: "
                                 << config.serialNumber ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        calibFileName = pt.get<string>("connection.calibFileName");
        //config.calibFileName = pt.get<uint8_t *>("connection.calibFileName");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read calibFileName: "
                                 << calibFileName;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        int32_t frameMode = pt.get<int32_t>("connection.frameMode",BTA_FrameModeDistAmp);
        config.frameMode = (BTA_FrameMode)frameMode;
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read frameMode: "
                                 << config.frameMode;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        bltstreamFilename = pt.get<string>("connection.bltstreamFilename");
        //config.calibFileName = pt.get<uint8_t *>("connection.calibFileName");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read bltstreamFilename: "
                                 << bltstreamFilename;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    /*try {
        //TODO Check
        //config.infoEvent = pt.get<FN_BTA_InfoEvent>("connection.infoEvent");
        //BOOST_LOG_TRIVIAL(debug) << "Read infoEvent: "
        //	<< config.infoEvent ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(info) << "Error getting parameters: " << e.what();
    }*/
    try {

        config.verbosity = pt.get<uint8_t>("connection.verbosity");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read verbosity: "
                                 << (int)config.verbosity;

    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        //TODO Check
        //config.frameArrived = pt.get<FN_BTA_FrameArrived>("connection.frameArrived");
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        config.frameQueueLength = pt.get<uint16_t>("connection.frameQueueLength");
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read frameQueueLength: "
                                 << config.frameQueueLength ;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        int32_t frameQueueMode = pt.get<int32_t>("connection.frameQueueMode",BTA_QueueModeDropOldest);
        BOOST_LOG_TRIVIAL(debug) << "Read frameQueueMode: "
                                 << config.frameQueueMode ;
        config.frameQueueMode = (BTA_QueueMode)frameQueueMode;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    try {
        int32_t deviceType = pt.get<int32_t>("connection.deviceType",config.deviceType);
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Read deviceType: "
                                 << config.deviceType;
        config.deviceType = (BTA_DeviceType)deviceType;
    } catch (std::exception &e) {
        BOOST_LOG_TRIVIAL(debug) << "BtaWrapper::parseConfig Error getting parameters: " << e.what();
    }
    return 0;
}

/*int BtaWrapper::setConfig(BTA_Config inConfig) {
    BTAinitConfig(&config);
    memcpy(config,inConfig,sizeof(BTA_Config));

}*/

int BtaWrapper::reConnect(){
    handle=0;
    if(connect())
        return 1;
    return 0;
}

int BtaWrapper::connect(){
    if (isConnected()) {
        BOOST_LOG_TRIVIAL(warning) << "The camera is already connected.";
        return -1;
    }
    config.uartPortName = (uint8_t *)uartPortName.c_str();

    config.calibFileName = (uint8_t *)calibFileName.c_str();
    config.bltstreamFilename = (uint8_t *)bltstreamFilename.c_str();

    config.infoEvent = infoEventCb;
    //config.frameArrived = frameArrived;
    BOOST_LOG_TRIVIAL(info) << "BtaWrapper::connect Read udpDataIpAddr: "
                            << (int)config.tcpDeviceIpAddr[0] << "."
                            << (int)config.tcpDeviceIpAddr[1] << "."
                            << (int)config.tcpDeviceIpAddr[2] << "."
                            << (int)config.tcpDeviceIpAddr[3];

    BOOST_LOG_TRIVIAL(info) << "BtaWrapper::connect Read udpDataIpAddr: "
                            << (int)config.udpDataIpAddr[0] << "."
                            << (int)config.udpDataIpAddr[1] << "."
                            << (int)config.udpDataIpAddr[2] << "."
                            << (int)config.udpDataIpAddr[3] << ".";

    status = BTAopen(&config, &handle);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning)
                << "BtaWrapper::connect BTAopen: Could not connect to the camera. status: "
                << status;
        return status;
    }
    BOOST_LOG_TRIVIAL(debug)
            << "BtaWrapper::connect Camera connected sucessfully. status: "
            << status;
    
    status = BTAgetDeviceInfo(handle, &deviceInfo);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "Could not get device info.";
        return -1;
    }
    BOOST_LOG_TRIVIAL(debug) << "Retrieved device info: \n"
                             << "deviceType: " << deviceInfo->deviceType << "\n"
                             << "serialNumber: " << deviceInfo->serialNumber << "\n"
                             << "firmwareVersionMajor: " << deviceInfo->firmwareVersionMajor << "\n"
                             << "firmwareVersionMinor: " << deviceInfo->firmwareVersionMinor << "\n"
                             << "firmwareVersionNonFunc: " << deviceInfo->firmwareVersionNonFunc
                             << "\n";

    device = deviceInfo->deviceType;
    BOOST_LOG_TRIVIAL(debug) << "Service running: " << (int)BTAisRunning(handle);
    BOOST_LOG_TRIVIAL(debug) <<"Connection up: " << (int)BTAisConnected(handle);
    
    return 0;

}
int BtaWrapper::disconnect() {

    if (deviceInfo != NULL) {
        status = BTAfreeDeviceInfo(deviceInfo);
        if (status != BTA_StatusOk) {
            BOOST_LOG_TRIVIAL(warning) << "Could not free device info. status: "
                                       << status;
        }
        deviceInfo = NULL;
    }
    //cout << "isConnected: " << isConnected() << endl;
    if (isConnected()) {
        status = BTAclose(&handle);
        if (status != BTA_StatusOk) {
            BOOST_LOG_TRIVIAL(warning) << "Could not disconnect. Status: "
                                       << status;
        }
        //Setting the handle to 0 as the bta_p100 keeps returning connected
        handle = 0;

    }
    return 0;
}

bool BtaWrapper::isConnected() const {
    //cout << "BTAisConnected(handle): " << (int)BTAisConnected(handle) << endl;
    if (BTAisConnected(handle))
        return 1;
    else
        return 0;
}

int BtaWrapper::capture(char * &buffer) {
    BTA_Frame *frame;
    int i;
    for (i=0; i <= retries; i++) {
        status = BTAgetFrame(handle, &frame, 100);
        if (status != BTA_StatusOk) {
            if (status == BTA_StatusTimeOut) { // no data available yet - sleep a little
                BTAsleep(2);
                continue;
            }
            BOOST_LOG_TRIVIAL(warning) << "Could no capture frame. status: " << status;
            continue;
        } else {
            break;
        }
    }
    if (i > retries) {
        BOOST_LOG_TRIVIAL(warning) << "Could not capture.";
        return 0;
    }

    
    //BOOST_LOG_TRIVIAL(debug) << "Frame captured";
    buffer = (char *)frame;
    /*
    BOOST_LOG_TRIVIAL(debug) << "frame size: " << sizeof(BTA_Frame);
    BOOST_LOG_TRIVIAL(debug) << "BTA_CHANNEL size: " << sizeof(BTA_Channel);
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionNonFunc: " << (int)frame->firmwareVersionNonFunc;
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionMinor: " << (int)frame->firmwareVersionMinor;
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionMajor: " << (int)frame->firmwareVersionMajor;
    BOOST_LOG_TRIVIAL(debug) << "frame->mainTemp: " << (float)frame->mainTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->ledTemp " << (float)frame->ledTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->genericTemp " << (float)frame->genericTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->frameCounter " << (float)frame->frameCounter;
    BOOST_LOG_TRIVIAL(debug) << "frame->timeStamp " << frame->timeStamp;
    BOOST_LOG_TRIVIAL(debug) << "frame->channelsLen: " << (int)frame->channelsLen;
    //buffer = (char *)malloc(sizeof(BTA_Frame));
    //memcpy(buffer,frame,sizeof(BTA_Frame));
    */
    return 1;
}

int BtaWrapper::registerOp(unsigned int reg) {
    uint32_t usValue;
    status = BTAreadRegister(handle, reg, &usValue, 0);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "Could read reg: status" << status;
        return status;
    }
    return usValue;
}

int BtaWrapper::registerOp(unsigned int reg, unsigned int data) {
    status = BTAwriteRegister(handle, reg, &data, 0);
    if (status != BTA_StatusOk) {
        BOOST_LOG_TRIVIAL(warning) << "Could write reg. status: " << status;
        return -1;
    }
    BOOST_LOG_TRIVIAL(debug) << "Register: " << reg << " set to: " << data;
    return 0;
}

char * BtaWrapper::loadFrame(char *data, std::string ext) {
    BTA_Frame *frame = (BTA_Frame *)malloc(sizeof(BTA_Frame));
    memcpy(frame,data,sizeof(BTA_Frame));

#ifdef CM_DEBUG
    namespace logging = boost::log;
    logging::core::get()->set_filter
            (logging::trivial::severity >= logging::trivial::debug);
#endif

    BOOST_LOG_TRIVIAL(debug) << "frame size: " << sizeof(BTA_Frame);
    BOOST_LOG_TRIVIAL(debug) << "BTA_CHANNEL size: " << sizeof(BTA_Channel);
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionNonFunc: " << (int)frame->firmwareVersionNonFunc;
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionMinor: " << (int)frame->firmwareVersionMinor;
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionMajor: " << (int)frame->firmwareVersionMajor;
    BOOST_LOG_TRIVIAL(debug) << "frame->mainTemp: " << (float)frame->mainTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->ledTemp " << (float)frame->ledTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->genericTemp " << (float)frame->genericTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->frameCounter " << (float)frame->frameCounter;
    BOOST_LOG_TRIVIAL(debug) << "frame->timeStamp " << (int)frame->timeStamp;
    cout << endl;

    //if (UNIX && ext == ".rw")
    memcpy(&frame->channelsLen,data+28,1);


    BOOST_LOG_TRIVIAL(debug) << "frame->channelsLen: " << (int)frame->channelsLen;
    cout << endl;

    //TODO channelsLen is nº of channels or the byte lenght??
    frame->channels = (BTA_Channel **)malloc((sizeof(BTA_Channel *) * frame->channelsLen));


    //TODO channelsLen is nº of channels or the byte lenght??
    int pos = sizeof(BTA_Frame);
    //if (UNIX && ext == ".rw")
    pos = 32;
    for (int i=0; i < frame->channelsLen; i++) {
        //BTA_Channel *test = ()(frame+pos)
        //TEST

        //frame->channels[i] = (BTA_Channel *)malloc(28);
        //memcpy(frame->channels[i],data+pos,28);

        frame->channels[i] = (BTA_Channel *)malloc(sizeof(BTA_Channel));
        memcpy(frame->channels[i],data+pos,sizeof(BTA_Channel));
        //memcpy(frame->channels[i],data+pos,28);



        BOOST_LOG_TRIVIAL(debug) << "frame size: " << sizeof(BTA_Frame);
        BOOST_LOG_TRIVIAL(debug) << "BTA_CHANNEL size: " << sizeof(BTA_Channel);
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->id: " << (int)frame->channels[i]->id;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->xRes: " << (int)frame->channels[i]->xRes;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->yRes: " << (int)frame->channels[i]->yRes;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->dataFormat: " << (int)frame->channels[i]->dataFormat;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->unit: " << (int)frame->channels[i]->unit;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->integrationTime: " << (int)frame->channels[i]->integrationTime;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->modulationFrequency: " << (int)frame->channels[i]->modulationFrequency;
        cout << endl;

        //TEST
        //pos+=28;
        pos+=sizeof(BTA_Channel);
        //if (UNIX && ext == ".rw")
        //pos -= ( sizeof(BTA_Channel) - 28 ); /Changed for arm, change if it fails for windows files
        pos -= ( sizeof(BTA_Channel) - 32 );
        int dataSize = sizeof(float);
        if (frame->channels[i]->dataFormat == BTA_DataFormatUInt16)
            dataSize = sizeof(unsigned short);
        if (frame->channels[i]->dataFormat == BTA_DataFormatUInt32)
            dataSize = sizeof(unsigned int);

        BOOST_LOG_TRIVIAL(debug) << (int)(dataSize * (frame->channels[i]->xRes * frame->channels[i]->yRes));
        dataSize*= frame->channels[i]->xRes * frame->channels[i]->yRes;

        frame->channels[i]->data = (uint8_t *)malloc(dataSize);
        memcpy(frame->channels[i]->data,data+pos,dataSize);

        pos+=dataSize;
    }
    BOOST_LOG_TRIVIAL(debug) << "frame loaded.";
    return (char *)frame;
}

char * BtaWrapper::serializeFrame(char *data, size_t &size) {
    BTA_Frame *frame =  (BTA_Frame *)data;
    vector<size_t> channels(frame->channelsLen);
    //size_t channels[frame->channelsLen];
    //channelsLen is nº of channels
    size = sizeof(BTA_Frame)+sizeof(BTA_Channel)*frame->channelsLen;

    BOOST_LOG_TRIVIAL(info) << "frame size: " << sizeof(BTA_Frame);
    BOOST_LOG_TRIVIAL(info) << "BTA_CHANNEL size: " << sizeof(BTA_Channel);
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionNonFunc " << (int)frame->firmwareVersionNonFunc;
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionMinor " << (int)frame->firmwareVersionMinor;
    BOOST_LOG_TRIVIAL(debug) << "frame->firmwareVersionMajor " << (int)frame->firmwareVersionMajor;
    BOOST_LOG_TRIVIAL(debug) << "frame->mainTemp " << (float)frame->mainTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->ledTemp " << (float)frame->ledTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->genericTemp " << (float)frame->genericTemp;
    BOOST_LOG_TRIVIAL(debug) << "frame->frameCounter " << (int)frame->frameCounter;
    BOOST_LOG_TRIVIAL(debug) << "frame->timeStamp " << (int)frame->timeStamp;
    BOOST_LOG_TRIVIAL(debug) << "frame->channelsLen: " << (int)frame->channelsLen;
    BOOST_LOG_TRIVIAL(debug) << "frame->sequenceCounter: " << (int)frame->sequenceCounter;

    for (int i=0; i < frame->channelsLen; i++) {
        int dataSize = sizeof(float);
        if (frame->channels[i]->dataFormat == BTA_DataFormatUInt16)
            dataSize = sizeof(unsigned short);
        else if (frame->channels[i]->dataFormat == BTA_DataFormatUInt32)
            dataSize = sizeof(unsigned int);
        else if (frame->channels[i]->dataFormat == BTA_DataFormatFloat32)
            dataSize = sizeof(unsigned int);
        else {
            BOOST_LOG_TRIVIAL(error) << "unknown dataFormat: " << (int)(frame->channels[i]->dataFormat);
        }

        channels[i] = frame->channels[i]->xRes*frame->channels[i]->yRes*dataSize;
        size+= channels[i];
    }

    char *raw = (char *)malloc(size);
    size_t count = sizeof(BTA_Frame);
    memcpy(raw,frame,sizeof(BTA_Frame));

    for (int i=0; i < frame->channelsLen; i++) {
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->id: " << (int)frame->channels[i]->id;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->xRes: " << (int)frame->channels[i]->xRes;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->yRes: " << (int)frame->channels[i]->yRes;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->dataFormat: " << (int)frame->channels[i]->dataFormat;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->unit: " << (int)frame->channels[i]->unit;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->integrationTime: " << frame->channels[i]->integrationTime;
        BOOST_LOG_TRIVIAL(debug) << "frame->channels[i]->modulationFrequency: " << frame->channels[i]->modulationFrequency;

        BOOST_LOG_TRIVIAL(debug) << "sizeof(BTA_Channel): " << sizeof(BTA_Channel);
        BOOST_LOG_TRIVIAL(debug) << "channels[i]: " << (int)channels[i];

        memcpy(raw+count,frame->channels[i],sizeof(BTA_Channel));

        count+=sizeof(BTA_Channel);

        size_t dataSize = channels[i];

        memcpy(raw+count,frame->channels[i]->data,dataSize);

        count+=dataSize;
    }
    return raw;
}

int BtaWrapper::freeFrame(char *data) {
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

int BtaWrapper::getDistances(float * &depth, int &size, char *data) {

    BTA_Frame *frame = (BTA_Frame *)data;

    void *disVoid;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    uint16_t xRes, yRes;
    //BOOST_LOG_TRIVIAL(debug) << "BTAgetDistances()";
    status = BTAgetDistances(frame, &disVoid, &dataFormat, &unit, &xRes, &yRes);

    if (status != BTA_StatusOk) {
        return -1;
    }

    if (size != xRes * yRes) {
        size = xRes * yRes;
        BOOST_LOG_TRIVIAL(debug) << "Size does not match! " << size ;
        BOOST_LOG_TRIVIAL(debug) << "xRes: " << xRes;
        BOOST_LOG_TRIVIAL(debug) << "yRes: " << yRes;

        return -1;
    }
    
    if (dataFormat == BTA_DataFormatUInt16) {
        //BOOST_LOG_TRIVIAL(debug) << "unit: " << unit;
        if (unit == BTA_UnitMillimeter) {
            unsigned short *distances = (unsigned short *)disVoid;

            for (int j=0; j< size ; j++) {
                if (distances[j] <= 0x0001 || distances[j] > 0xFA00)
                    depth[j] = 0.0;
                //depth[j] = std::numeric_limits<float>::quiet_NaN();
                else
                    depth[j] = (distances[j]/1000.f);
            }
            return 1;
        }
    } else if (dataFormat == BTA_DataFormatFloat32) {
        if (unit == BTA_UnitMeter) {
            float *distances = (float *)disVoid;

            for (int j=0; j< size ; j++) {
                if (distances[j] <= 0.1f || distances[j] > 20.0f)
                    depth[j] = std::numeric_limits<float>::quiet_NaN();
                else
                    depth[j] = distances[j];
                //BOOST_LOG_TRIVIAL(warning) << depth[j];
            }
            return 1;
        }
    } else {
        BOOST_LOG_TRIVIAL(warning) << "Unknown data format! " << dataFormat;
    }


    return -1;

}


int BtaWrapper::getAmplitudes(unsigned short * &amplitudes, int &size, char *data) {

    BTA_Frame *frame = (BTA_Frame *)data;
    void *amplVoid;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    uint16_t xRes, yRes;
    int sz = size;
    status = BTAgetAmplitudes(frame, &amplVoid,
                              &dataFormat, &unit,
                              &xRes, &yRes);

    if (status != BTA_StatusOk) {
        return -1;
    }

    if (sz != xRes * yRes) {
        size = xRes * yRes;
        BOOST_LOG_TRIVIAL(warning) << "Size does not match! " << size << " " << xRes << " " << yRes << " " << (xRes * yRes) << endl;
        BOOST_LOG_TRIVIAL(debug) << "xRes: " << xRes;
        BOOST_LOG_TRIVIAL(debug) << "yRes: " << yRes;

        return -1;
    }

    if (dataFormat == BTA_DataFormatUInt16) {
        if (unit == BTA_UnitUnitLess) {
            size = xRes * yRes;

            memcpy(amplitudes,amplVoid,sizeof(unsigned short)*size);

            return 1;
        }
    } else if (dataFormat == BTA_DataFormatFloat32) {
        //Keeping BTA_UnitMeter because of error in BtaApi v1.0.0
        if (unit == BTA_UnitUnitLess || unit == BTA_UnitMeter) {
            float *amp = (float *)amplVoid;
            //printf("Got amplitude data\n");
            size = xRes * yRes;

            for (int j=0; j< size ; j++) {
                amplitudes[j] = (unsigned short)amp[j];
            }
            return 1;
        }
    } else {
        BOOST_LOG_TRIVIAL(warning) << "Unknown data format! " << dataFormat;
    }

    return -1;

}

int BtaWrapper::reset() {
    return BTAsendReset(handle);
}

int BtaWrapper::getDisSize(char *data, int &x, int &y) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BTA_Frame *frame = (BTA_Frame *)data;
    uint16_t *distances;
    BTA_DataFormat dataFormat;
    BTA_Unit unit;
    //BOOST_LOG_TRIVIAL(debug) << "BTAgetDistances()";
    status = BTAgetDistances(frame, (void **)&distances, &dataFormat,
                             &unit, (uint16_t*)&x, (uint16_t*)&y);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status == BTA_StatusOk) {
        return 1;
    }
    return -1;
}

int BtaWrapper::getAmpSize(char *data, int &x, int &y) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BTA_Frame *frame = (BTA_Frame *)data;

    BTA_DataFormat dataFormat;
    uint16_t *amplitudes;
    BTA_Unit unit;
    //BOOST_LOG_TRIVIAL(debug) << "BTAgetAmplitudes()";
    status = BTAgetAmplitudes(frame, (void **)&amplitudes, &dataFormat,
                              &unit, (uint16_t*)&x, (uint16_t*)&y);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status == BTA_StatusOk) {
        return 1;
    }
    return -1;
}

int BtaWrapper::getFrameTime(char *data, unsigned int &timeStamp) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BTA_Frame *frame = (BTA_Frame *)data;
    if (frame != NULL) {
        BOOST_LOG_TRIVIAL(debug) << "timeStamp: " << frame->timeStamp;
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

float mainTemp;                   //<<< Main-board/processor temperature sensor in degree Celcius
float ledTemp;                    //<<< Led-board temperature sensor in degree Celcius
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


unsigned int BtaWrapper::getIntegrationTime(){
    unsigned int it;
    status = BTAgetIntegrationTime(handle, &it);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return it;
}
float BtaWrapper::getFrameRate() {
    float fr;
    status = BTAgetFrameRate(handle, &fr);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return fr;
}

unsigned long BtaWrapper::getModulationFrequency() {
    unsigned int it;
    status = BTAgetModulationFrequency(handle, &it);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return it;
}

int BtaWrapper::setIntegrationTime(unsigned int it) {
    status = BTAsetIntegrationTime(handle, it);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return 1;
}

int BtaWrapper::setFrameRate(float fr) {
    status = BTAsetFrameRate(handle, fr);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return 1;
}

int BtaWrapper::setModulationFrequency(unsigned long mf) {
    status = BTAsetModulationFrequency(handle, mf);
    BOOST_LOG_TRIVIAL(debug) << "Status: " << status;
    if (status != BTA_StatusOk) {
        return -1;
    }
    return 1;
}


int BtaWrapper::startGrabbing(std::string filename) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BTA_GrabbingConfig grabbingConfig;
    status = BTAinitGrabbingConfig(&grabbingConfig);
    errorHandling(status);
    //TODO check filename; renaming; etc..
    grabbingConfig.filename = (unsigned char*)filename.c_str();
    status = BTAstartGrabbing(handle, &grabbingConfig);
    errorHandling(status);
    //TODO check status;
    return 1;
}

int BtaWrapper::stopGrabbing() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    status = BTAstartGrabbing(handle, 0);
    errorHandling(status);
    return 1;
}

int BtaWrapper::saveRaw(string fileName, char *data) {

    size_t data_serialized_size;
    char *data_serialized = serializeFrame(data,data_serialized_size);

    char *raw = new char[sizeof(FrameHeader)+data_serialized_size];

    FrameHeader header;
    header.manufacturer = manufacturer;
    //BOOST_LOG_TRIVIAL(debug) << "camera->getDevice() " << camera->getDevice();
    header.device = device;
    //BOOST_LOG_TRIVIAL(debug) << "header.device " << header.device;
    header.lenght = static_cast<int>(data_serialized_size);

    memcpy(raw,&header,sizeof(FrameHeader));
    memcpy(raw+sizeof(FrameHeader),data_serialized,data_serialized_size);
    free(data_serialized);
    BOOST_LOG_TRIVIAL(debug) << "Saving file: " << fileName << ", size: " << data_serialized_size + sizeof(FrameHeader);
    //Frame *test = (Frame *)raw;
    //BOOST_LOG_TRIVIAL(debug) << "Tests frame: " << test->device;
    ofstream f;
    f.open (fileName.c_str(), ios::out | ios_base::binary );
    f.write ( raw, sizeof(FrameHeader)+data_serialized_size);
    f.close();
    BOOST_LOG_TRIVIAL(debug) << "Saved file: " << fileName;
    free(raw);
    return 0;
}

char * BtaWrapper::loadRaw(string rawFile) {

    FILE* raw = fopen(rawFile.c_str(), "rb");
    if (raw == NULL)
    {
        BOOST_LOG_TRIVIAL(warning) << "Could not open file: " << rawFile;
        return NULL;
    }

    FrameHeader header;

    size_t ret;
    ret = fread(&header, sizeof(FrameHeader), 1, raw);
    if (ret == 0) {
        BOOST_LOG_TRIVIAL(warning) << "Could not read file: " << rawFile;
        return NULL;
    }

    /*if (checkRaw(header)){
        BOOST_LOG_TRIVIAL(warning) << "Raw frame does not match sensor device.";
        return NULL;
    }
    BOOST_LOG_TRIVIAL(debug) << "sizeof(FrameHeader): " << sizeof(FrameHeader);*/


    char *frame_data = new char[header.lenght];
    ret = fread(frame_data, sizeof(char),header.lenght, raw);
    if (ret == 0) {
        BOOST_LOG_TRIVIAL(warning) << "Could not read file: " << rawFile;
        return NULL;
    }

    BOOST_LOG_TRIVIAL(debug) << "Raw frame read.";
    fclose (raw);

    string ext = rawFile.substr(rawFile.find_last_of("."),rawFile.length() - rawFile.find_last_of("."));

    char *retu = loadFrame(frame_data, ext);
    delete[] frame_data;
    return retu;
}

BTA_DeviceType BtaWrapper::getDeviceType() const
{
    return config.deviceType;
}

void BtaWrapper::setDeviceType(const BTA_DeviceType &value) {
    config.deviceType = value;
}

int BtaWrapper::getLibParam(int &param, float &data){
    return BTAgetLibParam(handle, (BTA_LibParam)param, &data);
}

int BtaWrapper::setLibParam(int &param, float &data) {
    return BTAsetLibParam(handle, (BTA_LibParam)param, data);
}
