#include <stdlib.h>

#include <boost/algorithm/string/trim.hpp>
#include <boost/any.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>
#include <ctime>
#include <iostream>
#include <map>

// TODO MOVE TO OPENCV
//#include <pcl/common/transforms.h>
//#include <pcl/common/angles.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <toffy/bta/bta.hpp>
#include <toffy/filter_helpers.hpp>

#define RECONNECT 10
static int retries = 0;

using namespace toffy;
using namespace toffy::capturers;
using namespace cv;
using namespace std;
namespace fs = boost::filesystem;

std::size_t Bta::_filter_counter = 1;
const std::string Bta::id_name = "bta";

Bta::Bta()
    : CapturerFilter(Bta::id_name, _filter_counter),
      distsSize(0),
      width(0),
      height(0),
      globalOfs(0),
      eth0Config(6),
      hasEth0Config(false),
      interfaceConfig(0),
      hasIfConfig(false),
      hasGlobalOfs(false),
      modulationFreq(-1),
      _out_depth("depth"),
      _out_ampl("ampl"),
      _out_mf("mf"),
      _out_it("it"),
      _out_fc("fc"),
      _out_ts("ts"),
      _out_mt("mt"),
      _out_lt("lt"),
      _out_gt("lt")
{
    _filter_counter++;
    sensor = new BtaWrapper();
    fileExt(RAWFILE);
}

Bta::~Bta()
{
    if (((capturers::CapturerFilter*)this)->save() && isConnected())
        sensor->stopGrabbing();
    disconnect();

    delete sensor;
}

int Bta::loadConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();
    const boost::property_tree::ptree& bta = pt.get_child(type());

    Filter::loadConfig(pt);

    _out_depth = pt.get<string>("outputs.depth", _out_depth);
    _out_ampl = pt.get<string>("outputs.ampl", _out_ampl);
    _out_mf = pt.get<string>("outputs.mf", _out_mf);
    _out_it = pt.get<string>("outputs.it", _out_it);
    _out_fc = pt.get<string>("outputs.fc", _out_fc);
    _out_ts = pt.get<string>("outputs.ts", _out_ts);
    _out_mt = pt.get<string>("outputs.mt", _out_mt);
    _out_lt = pt.get<string>("outputs.lt", _out_lt);
    _out_gt = pt.get<string>("outputs.gt", _out_gt);

    sensor->parseConfig(bta);

    bool present;
    present = pt_optional_get(bta, "options.dynamicOutputs", dynOutputs);
    if (present) {
        BOOST_LOG_TRIVIAL(debug)
            << __FUNCTION__ << " " << __LINE__ << " dynOutputs? " << dynOutputs;
    }

    present =
        pt_optional_get(bta, "options.modulationFrequency", modulationFreq);
    if (present) {
        BOOST_LOG_TRIVIAL(debug)
            << __FUNCTION__ << " " << __LINE__ << " modulationFreq set to  "
            << modulationFreq;
    } else {
        modulationFreq = -1;
    }
    present = pt_optional_get(bta, "options.globalOffset", globalOfs);
    hasGlobalOfs = present;
    if (present) {
        BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << __LINE__
                                 << " globalOffset set to  " << globalOfs;
    }

    present = pt_optional_get(bta, "options.eth0Config", eth0Config);
    hasEth0Config = present;

    present = pt_optional_get(bta, "options.interfaceConfig", interfaceConfig);
    hasIfConfig = present;

    boost::optional<bool> playbk = bta.get_optional<bool>("playback");
    if (playbk.is_initialized()) {
        playback(*playbk);
        BOOST_LOG_TRIVIAL(debug)
            << __FUNCTION__ << " " << __LINE__ << " playback?" << *playbk;
    }

    boost::optional<bool> autoconnect = bta.get_optional<bool>("autoconnect");
    if (autoconnect.is_initialized()) {
        if (*autoconnect == true) {
            if (sensor->getDeviceType() == BTA_DeviceTypeGenericBltstream &&
                bta_stream)
                playback(true);
            else
                connect();
        }
    }
    if (!this->playback()) {
        boost::optional<int> it = bta.get_optional<int>("it");
        if (it.is_initialized() && isConnected()) {
            sensor->setIntegrationTime(*it);
        }
        boost::optional<float> fr = bta.get_optional<float>("fr");
        if (fr.is_initialized() && isConnected()) {
            sensor->setFrameRate(*fr);
        }
    }
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << __LINE__ << " blts? "
                             << sensor->getBltstream();
    if (sensor->getBltstream().length() > 0) {
        loadPath(sensor->getBltstream());
        playback(true);
    }

    return 1;
}

void Bta::updateConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    CapturerFilter::updateConfig(pt);

    _out_depth = pt.get<string>("outputs.depth", _out_depth);
    _out_ampl = pt.get<string>("outputs.ampl", _out_ampl);
    _out_mf = pt.get<string>("outputs.mf", _out_mf);
    _out_it = pt.get<string>("outputs.it", _out_it);
    _out_fc = pt.get<string>("outputs.fc", _out_fc);
    _out_ts = pt.get<string>("outputs.ts", _out_ts);
    _out_mt = pt.get<string>("outputs.mt", _out_mt);
    _out_lt = pt.get<string>("outputs.lt", _out_lt);
    _out_gt = pt.get<string>("outputs.gt", _out_gt);

    camType = pt.get<string>("options.camera", "P230");
    if (camType == "P230") {
        cam.reset(new cam::P230());
    } else if (camType == "M520") {
        cam.reset(new cam::M520());
    } else {
        BOOST_LOG_TRIVIAL(info) << "assuming default camera type; please set options.camera to override!";
        cam.reset(new cam::P230());
    }
    update = true;
}

boost::property_tree::ptree Bta::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = CapturerFilter::getConfig();

    pt.put("outputs.depth", _out_depth);
    pt.put("outputs.ampl", _out_ampl);
    pt.put("outputs.mf", _out_mf);
    pt.put("outputs.it", _out_it);
    pt.put("outputs.fc", _out_fc);
    pt.put("outputs.ts", _out_ts);
    pt.put("outputs.mt", _out_mt);
    pt.put("outputs.lt", _out_lt);
    pt.put("outputs.gt", _out_gt);

    pt.put("options.camera", camType);

    return pt;
}

bool Bta::filter(const Frame& in, Frame& out)
{
    char* data = NULL;

    using namespace boost::posix_time;

    ptime start = microsec_clock::local_time();
    boost::posix_time::time_duration diff;

    // Check if the device is connected through the blttofapi.
    // If not try to reconnect. After 10 tries, stop the toffy.
    // TODO makes this optional as some application do not need to stop toffy
    // completely
    if (!isConnected()) {
        retries++;
        if (connect() < 0) {
            if (retries > RECONNECT) {
                BOOST_LOG_TRIVIAL(error)
                    << "Camera not reachable after " << RECONNECT
                    << "tries. Stopping toffy.";
                exit(EXIT_FAILURE); // @TODO report failure to filterBank?
            } else {
                BOOST_LOG_TRIVIAL(warning)
                    << "Could not reconnect to device. Retry: " << retries;
                return false;
            }
            sleep(1); // BAD SYNC CALL, but without camera connection, there's nothing to do...
        } else {
            retries = 0;
        }
    }

    //diff = boost::posix_time::microsec_clock::local_time() - start;
    //BOOST_LOG_TRIVIAL(debug)
    //    << "duration pre-capture: " << diff.total_milliseconds()
    //    << " playback? " << this->playback() << " bta_stream " << bta_stream;

    // Checks if there is a data file to load.
    if (this->playback()) {
        if (!bta_stream) {
            // PlayBack in r/rw files
            if (in.hasKey("backward") || backward()) {
                cnt(cnt() - 1);
                if (cnt() < beginFile()) cnt(endFile());
            } else {
                cnt(cnt() + 1);
                if (cnt() > endFile()) cnt(beginFile());
            }
            BOOST_LOG_TRIVIAL(debug)
                << ((CapturerFilter*)this)->loadPath() + "/" +
                       boost::lexical_cast<std::string>(cnt()) + fileExt();
            data = sensor->loadRaw(((CapturerFilter*)this)->loadPath() + "/" +
                                   boost::lexical_cast<std::string>(cnt()) +
                                   fileExt());
        } else {
            BOOST_LOG_TRIVIAL(debug) << "bta::filter " << __LINE__
                                     << " cap async? " << sensor->isAsync();
            if (sensor->isAsync()) {
                BOOST_LOG_TRIVIAL(debug)
                    << "bta::filter " << __LINE__ << " cap async... "
                    << sensor->isAsync();
                data = (char*)sensor->waitForNextFrame();
                BOOST_LOG_TRIVIAL(debug) << "bta::filter " << __LINE__
                                         << " cap async! " << sensor->isAsync();

            } else {
                BOOST_LOG_TRIVIAL(debug)
                    << "bta::filter " << __LINE__ << " what should I do? ";
            }
        }
    } else {  // live connection
        if (sensor->isAsync()) {
            data = (char*)sensor->waitForNextFrame();
        } else {
            BOOST_LOG_TRIVIAL(debug)
                << "bta::filter " << __LINE__ << " what should I do? ";
        }
    }

    diff = boost::posix_time::microsec_clock::local_time() - start;
    //BOOST_LOG_TRIVIAL(debug)
    //    << "duration pre-capture:2 " << diff.total_microseconds();

    if (!data && !sensor->capture(data)) {
        BOOST_LOG_TRIVIAL(warning) << "Could not capture from sensor.";
#ifdef BTA_P100
        // Issue in bta lib. We stop the application
        BOOST_LOG_TRIVIAL(error) << "Camera not reacheable. Stopping the app.";
        exit(EXIT_FAILURE);
#endif
        // retries++;
        // if (retries > RETRY) {
        disconnect();
        // retries = 0;
        // }
        return false;
    }

    // BOOST_LOG_TRIVIAL(info) << "Start bta filter.";
    unsigned int mf, it;
    sensor->getFrameRef(data, mf, it);
    out.addData(_out_mf, mf);
    out.addData(_out_it, it);

    sensor->getFrameCounter(data, it);
    out.addData(_out_fc, it);
    out.addData("fc2", it);

    // The timestamp are unlogic.
    sensor->getFrameTime(data, it);
    out.addData(_out_ts, it);

    // Temps
    static float t;
    sensor->getMainTemp(data, t);
    out.addData(_out_mt, t);
    sensor->getLedTemp(data, t);
    out.addData(_out_lt, t);
    sensor->getGenericTemp(data, t);
    out.addData(_out_gt, t);

    if (!out.hasKey(CAM_SLOT)) {
        out.addData(CAM_SLOT, cam);
    }

    diff = boost::posix_time::microsec_clock::local_time() - start;
    //BOOST_LOG_TRIVIAL(debug) << "duration data: " << diff.total_microseconds();

    if (dynOutputs) {
        this->setOutputsDynamic(in, out, start, data);
    } else {
        this->setOutputsClassic(in, out, start, data);
    }

    diff = boost::posix_time::microsec_clock::local_time() - start;
    //BOOST_LOG_TRIVIAL(debug) << "duration free: " << diff.total_microseconds();

    return true;
}

int Bta::connect()
{
    int result = sensor->connect();
    if (result >= 0) {
        if (playback()) {
            float f = 0;
            int param = BTA_LibParamStreamPos;
            // sensor->setLibParam(param, f);
            param = BTA_LibParamStreamTotalFrameCount;
            sensor->getLibParam(param, f);
            cout << "BTA_LibParamStreamTotalFrameCount: " << f << endl;
            // endFile(f-1);
            // beginFile(0);
            // cnt(beginFile()-1);
        } else if (((CapturerFilter*)this)->save() && bta_stream) {
            BOOST_LOG_TRIVIAL(debug) << "strPath(): " << strPath();
            sensor->startGrabbing(strPath());
        }
        if (!playback()) {
            if (modulationFreq > 0) {
                sensor->setModulationFrequency(modulationFreq);
            }
            if (hasGlobalOfs) {
                sensor->setGlobalOffset(globalOfs);
            }
            if (hasEth0Config) {
                sensor->writeRegister(0x0240, eth0Config);
            }
            if (hasIfConfig) {
                sensor->writeRegister(0x00fa, interfaceConfig);
            }
            
            int freq = sensor->getModulationFrequency();
            int it = sensor->getIntegrationTime();
            float ofs = sensor->getGlobalOffset();
            BOOST_LOG_TRIVIAL(debug) << "ModulationFrequency: " << freq;
            BOOST_LOG_TRIVIAL(debug) << "IntegrationTime: " << it;
            BOOST_LOG_TRIVIAL(debug) << "Global offset: " << ofs;

            // if (sensor->hasChannels) {
            //     sensor->setChannels();
            // }
        }
    }
    return result;
}

int Bta::disconnect()
{
    int result = sensor->disconnect();
    if (result > 0 && ((CapturerFilter*)this)->save() && bta_stream)
        sensor->stopGrabbing();
    return result;
}

bool Bta::isConnected() { return sensor->isConnected(); }

void Bta::playback(const bool& pb)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BOOST_LOG_TRIVIAL(debug) << "bta_stream: " << bta_stream;
    BOOST_LOG_TRIVIAL(debug) << "pb: " << pb;
    if (pb == true && bta_stream) {
        sensor->setDeviceType(BTA_DeviceTypeGenericBltstream);
        /*if (sensor->getDeviceType() != BTA_DeviceTypeGenericBltstream) {
        BOOST_LOG_TRIVIAL(warning) << "Trying to read a bltstream file but the
    deviceType is not set to BTA_DeviceTypeGenericBltstream (15)!!!"; return;
    }*/

        sensor->setBltstream(((CapturerFilter*)this)->loadPath());

        /*
    if (connect() >= 0) {
        float f = 0;
        int param = BTA_LibParamStreamPos;
        sensor->setLibParam(param, f);
        param = BTA_LibParamStreamTotalFrameCount;
        sensor->getLibParam(param, f);
        cout << "BTA_LibParamStreamTotalFrameCount: " << f << endl;
        endFile(f-1);
        beginFile(0);
        cnt(beginFile()-1);
    }*/
    } else if (pb == true && !bta_stream) {
        return CapturerFilter::playback(pb);
    } else if (pb == false && bta_stream) {
        sensor->setBltstream("");
        disconnect();
    } else if (pb == false && !bta_stream) {
        return CapturerFilter::playback(pb);
    }
    CapturerFilter::playback(pb);
    return;
}

int Bta::loadPath(const std::string& newPath)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    fs::path fsPath(newPath);
    BOOST_LOG_TRIVIAL(debug) << "fsPath.extension(): " << fsPath.extension();
    if (fs::is_regular_file(fsPath) && fsPath.extension() == ".bltstream") {
        if (!fs::exists(fsPath)) {
            BOOST_LOG_TRIVIAL(warning) << "bta::loadPath(): Given path ["
                                       << newPath << "] does not exist.";
            return -1;
        }
        setLoadPath(newPath);
        if (isConnected()) {
            this->disconnect();
        }

        bta_stream = true;
        return 1;
    } else {
        bta_stream = false;
        BOOST_LOG_TRIVIAL(debug) << __LINE__ << "Bta::loadPath here";
        return CapturerFilter::loadPath(newPath);
    }
}

void Bta::savePath(const std::string& newPath)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    fs::path fsPath(newPath);
    cout << "is regular: " << fs::is_regular_file(fsPath) << endl;
    if (fsPath.extension() == ".bltstream") {
        setSavePath(newPath);
        // bta_stream = true;
        return;
    } else {
        // bta_stream = false;
        return CapturerFilter::savePath(newPath);
    }
}

void Bta::save(const bool& save)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    if (save == true) {
        cout << "PATH::: " << getSavePath() << endl;
        long int _saveTimeStamp;
#if defined(MSVC)
        using namespace boost::posix_time;
        ptime pt(second_clock::local_time());
        static ptime epoch(boost::gregorian::date(1970, 1, 1));
        time_duration diff(pt - epoch);
        _saveTimeStamp = diff.ticks() / diff.ticks_per_second();
#else
        time(&_saveTimeStamp);
#endif
        // if (!getSavePath().empty() && fs::path(getSavePath()).extension() ==
        // ".bltstream") { cout << "PATH::: " << saveFolder() << endl;
        fs::path newPath(getSavePath());
        // string _strPath = savePath + string("/") + id() + string("/") +
        // boost::lexical_cast<std::string>(_saveTimeStamp);
        BOOST_LOG_TRIVIAL(debug) << newPath.string();
        if (tsd()) {
            newPath /= boost::lexical_cast<std::string>(_saveTimeStamp);
        }
        newPath /= name();
        BOOST_LOG_TRIVIAL(debug) << newPath.string();
        try {
            fs::create_directories(fs::absolute(newPath));
        } catch (const fs::filesystem_error& e) {
            BOOST_LOG_TRIVIAL(warning)
                << "Could not create folder: " << strPath()
                << "; Reason: " << e.code().message();
            setSave(false);
            return;
        }

        // newPath /= getSavePath();

        if (fs::path(getSavePath()).extension() != ".bltstream") {
            newPath /=
                boost::lexical_cast<std::string>(_saveTimeStamp) + ".bltstream";
        }

        strPath(newPath.string());

        cout << "PATH::: " << strPath() << endl;
        if (this->isConnected()) sensor->startGrabbing(strPath());
        setSave(save);
    } else {
        if (this->isConnected()) sensor->stopGrabbing();
        return CapturerFilter::save(save);
    }
}

void Bta::setOutputsClassic(const Frame& in, Frame& out,
                            const boost::posix_time::ptime& start, char* data)
{
    boost::posix_time::time_duration diff;
    int size = distsSize, x = 0, y = 0;

    matPtr a, d;
    if (!out.hasKey(_out_depth) || size != distsSize) {
        // first iteration - initialize depth matrix
        sensor->getDisSize(data, x, y);

        width = x;
        height = y;
        size = distsSize = width * height;

        // initialize depth matrix ...:
        d.reset(new cv::Mat(height, width, CV_32F));
        out.addData(_out_depth, d);

        a.reset(new cv::Mat(height, width, CV_16U));
        out.addData(_out_ampl, a);

    } else {
        d = in.getMatPtr(_out_depth);
        a = in.getMatPtr(_out_ampl);
        x = width;
        y = height;
    }

    float* f = d->ptr<float>();
    sensor->getDistances(f, distsSize, data);

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration depth: " << diff.total_microseconds();

    // sensor->getAmpSize(data, x, y);
    // unsigned short *amplitude= NULL;

    unsigned short* da = a->ptr<unsigned short>();
    sensor->getAmplitudes(da, size, data);

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration ampl: " << diff.total_microseconds();

    if (flip()) {
        cv::flip(*a, *a, -1);
        cv::flip(*d, *d, -1);
    } else {
        if (flip_x()) {
            cv::flip(*a, *a, 1);
            cv::flip(*d, *d, 1);
        }
        if (flip_y()) {
            cv::flip(*a, *a, 0);
            cv::flip(*d, *d, 0);
        }
    }
    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration flip: " << diff.total_microseconds();

    out.addData(_out_depth, d);
    out.addData(_out_ampl, a);

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration add: " << diff.total_microseconds();

    if (sensor->isAsync()) {
        // nothing to do
    } else {
        cout << "FREEFRAME!!!" << endl;
        // sensor->freeFrame(data);
        data = NULL;
    }
}

void Bta::setOutputsDynamic(const Frame& /*in*/, Frame& out,
                            const boost::posix_time::ptime& /*start*/,
                            char* data)
{
    BTA_Frame* frame = (BTA_Frame*)data;
    for (int i = 0; i < frame->channelsLen; i++) {
        BTA_Channel* chan = frame->channels[i];
        if (!chan) {
            break;
        }
        if (i == 0) {
            width = chan->xRes;
            height = chan->yRes;
        }
        std::string name = getChannelTypeName(chan->id);
        std::string sel = "unknown";
        if (sensor->hasChannels) {
            sel = sensor->channelSelectionName(i);
        }
        // BOOST_LOG_TRIVIAL(debug)
        //     << "dynOut[" << i << "] " << name << " " << sel << " " <<
        //     chan->dataFormat << " "
        //     << chan->xRes << "x" << chan->yRes << " " << chan->dataLen;

        if (sensor->hasChannels) {
            // check for data type vs. channel name
            if (sensor->hasChannels &&
                name ==
                    "color") {  // override color type with actual channel name
                // BOOST_LOG_TRIVIAL(debug) << " channel nameset name " << name
                // << " to sel " << sel;
                name = sel;
            } else {
                if (name != sel) {
                    BOOST_LOG_TRIVIAL(warning)
                        << " channel name mismatch - check your config! name "
                        << name << " sel " << sel;
                }
            }
        }

        matPtr d;
        if (out.hasKey(name)) {
            d = out.getMatPtr(name);
        }
        switch (chan->dataFormat) {
            case BTA_DataFormatUInt8: {
                if (!d.get()) {
                    d.reset(new cv::Mat(height, width, CV_8UC1));
                }
                short* f = d->ptr<short>();
                memcpy(f, chan->data, chan->dataLen);
                break;
            }
            case BTA_DataFormatUInt16: {
                if (!d.get()) {
                    d.reset(new cv::Mat(height, width, CV_16UC1));
                }
                short* f = d->ptr<short>();
                memcpy(f, chan->data, chan->dataLen);
                break;
            }
            case BTA_DataFormatFloat32: {
                if (!d.get()) {
                    d.reset(new cv::Mat(height, width, CV_32F));
                }
                float* f = d->ptr<float>();
                memcpy(f, chan->data, chan->dataLen);
                break;
            }
            case BTA_DataFormatSInt16: {
                if (!d.get()) {
                    d.reset(new cv::Mat(height, width, CV_16SC1));
                }
                short* f = d->ptr<short>();
                memcpy(f, chan->data, chan->dataLen);
                break;
            }
            case BTA_DataFormatYuv422: {
                // BOOST_LOG_TRIVIAL(debug) << "img yuv data! " << (width *
                // height)
                //                          << " " << chan->dataLen;
                if (!d.get()) {
                    d.reset(new cv::Mat(height, width, CV_8UC3));
                }
                Mat input(height, width, CV_8UC2, chan->data);

                cvtColor(input, *d, COLOR_YUV2BGR_UYVY);
                break;
            }
            case BTA_DataFormatYuv444UYV: {
                // BOOST_LOG_TRIVIAL(debug) << "img yuv444uyv data! " << (width
                // * height)
                //                          << " " << chan->dataLen;
                if (!d.get()) {
                    d.reset(new cv::Mat(height, width, CV_8UC3));
                }
                // @TODO optimize
                unsigned char* ptr = chan->data;
                for (unsigned int i = 0; i < chan->dataLen; i += 3) {
                    unsigned char u = ptr[0];
                    unsigned char y = ptr[1];
                    unsigned char v = ptr[2];
                    ptr[0] = y;
                    ptr[1] = u;
                    ptr[2] = v;
                    ptr += 3;
                }
                Mat input(height, width, CV_8UC3, chan->data);
                cvtColor(input, *d, COLOR_YUV2BGR);

                // debug:
                // imshow("img", *d);

                break;
            }
            default:
                BOOST_LOG_TRIVIAL(warning) << "unkown data format! IMPLEMENT 0x"
                                           << hex << chan->dataFormat << dec;
                continue;
        }

        out.addData(name, d);
    }
}
