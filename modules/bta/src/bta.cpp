#include <map>
#include <ctime>
#include <iostream>

#include <boost/log/trivial.hpp>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
//TODO MOVE TO OPENCV
//#include <pcl/common/transforms.h>
//#include <pcl/common/angles.h>

#include <toffy/bta/bta.hpp>
/*
#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/core.hpp>
#else
#  include <opencv2/core/core.hpp>
#endif
*/

#include <stdlib.h>

#define RECONNECT 10
static int retries = 0;

using namespace toffy;
using namespace toffy::capturers;
using namespace cv;
using namespace std;
namespace fs = boost::filesystem;

std::size_t Bta::_filter_counter = 1;
const std::string Bta::id_name = "bta";

Bta::Bta(): CapturerFilter(Bta::id_name,_filter_counter),
    _out_depth("depth"), _out_ampl("ampl"), _out_mf("mf"), _out_it("it"),
    _out_fc("fc"), _out_ts("ts"), _out_mt("mt"), _out_lt("lt"), _out_gt("lt")
{
    _filter_counter++;
    sensor = new BtaWrapper();
    fileExt(RAWFILE);
}

Bta::~Bta(){
    if ( ((capturers::CapturerFilter*)this)->save() && isConnected())
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

    boost::optional<bool> autoconnect = bta.get_optional<bool>("autoconnect");
    if (autoconnect.is_initialized()) {
        if (*autoconnect == true) {
            if (sensor->getDeviceType() == BTA_DeviceTypeGenericBltstream && bta_stream)
                playback(true);
            else
                connect();
        }
    }

    boost::optional<int> it = bta.get_optional<int>("it");
    if (it.is_initialized() && isConnected()) {
        sensor->setIntegrationTime(*it);
    }
    boost::optional<float> fr = bta.get_optional<float>("fr");
    if (fr.is_initialized() && isConnected()) {
        sensor->setFrameRate(*fr);
    }

    return 1;
}


void Bta::updateConfig(const boost::property_tree::ptree &pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ <<  " " << id();

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

    return pt;
}

bool Bta::filter(const Frame &in, Frame& out) {
    //BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();
    char * data = NULL;

    using namespace boost::posix_time;

    ptime start = microsec_clock::local_time();
    boost::posix_time::time_duration diff;

    // Check if the device is connected through the blttofapi.
    // If not try to reconnect. After 10 tries, stop the toffy.
    // TODO makes this optional as some application do not need to stop toffy completely
    if (!isConnected()) {
        retries++;
        if (connect() < 0) {
            if (retries > RECONNECT) {
                BOOST_LOG_TRIVIAL(warning) << "Camera not reachable after "
                                           << RECONNECT
                                           << "tries. Stopping toffy.";
                exit(EXIT_FAILURE);
            } else {
                BOOST_LOG_TRIVIAL(warning) << "Could not reconnect to device. Retry: "
                                           << retries;
                return false;
            }
        } else {
            retries = 0;
        }
    }

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration pre-capture: " << diff.total_milliseconds();

    // Checks if there is a data file to load.
    if ( this->playback() ) {
        if (!bta_stream) {
            // PlayBack in r/rw files
            if (in.hasKey("backward") || backward()) {
                cnt(cnt()-1);
                if (cnt() < beginFile())
                    cnt(endFile());
            } else {
                cnt(cnt()+1);
                if (cnt() > endFile())
                    cnt(beginFile());
            }
            BOOST_LOG_TRIVIAL(debug) << ((CapturerFilter*)this)->loadPath() +
                                        "/" +
                                        boost::lexical_cast<std::string>(cnt()) +
                                        fileExt();
            data = sensor->loadRaw(
                        ((CapturerFilter*)this)->loadPath() + "/" +
                        boost::lexical_cast<std::string>(cnt()) +
                        fileExt());
// #if !defined(BTA_P100) && !defined(BTA_ETH)
//         } else /*if (bta_stream)*/ {
//             //diff = boost::posix_time::microsec_clock::local_time() - start;
//             //BOOST_LOG_TRIVIAL(info) << "duration start: " << diff.total_milliseconds();
//             //Plaback with btastream
//             static float f;
//             int param = BTA_LibParamStreamAutoPlaybackSpeed;
//             //cout << "get: " << sensor->getLibParam(param, f) << endl;
//             //cout << "value: " << f << endl;
//             if (in.hasKey("backward") || backward()) {
//                 //f = -1;
//                 /*param = BTA_LibParamStreamPosIncrement;
//         sensor->setLibParam(param, f);*/
//                 if (cnt() <= beginFile()) {
//                     cnt(endFile());
//                     f = cnt();
//                     param = BTA_LibParamStreamPos;
//                     sensor->setLibParam(param, f);
//                 } else
//                     cnt(cnt()-1);
//             } else {
//                 //diff = boost::posix_time::microsec_clock::local_time() - start;
//                 //BOOST_LOG_TRIVIAL(info) << "duration start 2: " << diff.total_milliseconds();
//                 //f = 1;
//                 /*param = BTA_LibParamStreamPosIncrement;
//         sensor->setLibParam(param, f);*/
//                 cout << "cnt(): " << cnt() << endl;
//                 if (cnt() >= endFile()) {
//                     cnt(beginFile());
//                     f = cnt();
//                     param = BTA_LibParamStreamPos;
//                     sensor->setLibParam(param, f);
//                 } else
//                     cnt(cnt()+1);
//             }
//             f = cnt();
//             //cout << "cnt: " << f << endl;
//             //diff = boost::posix_time::microsec_clock::local_time() - start;
//             //BOOST_LOG_TRIVIAL(info) << "duration before set param: " << diff.total_milliseconds();
//             param = BTA_LibParamStreamPos;
//             sensor->setLibParam(param, f);

//             diff = boost::posix_time::microsec_clock::local_time() - start;
//             BOOST_LOG_TRIVIAL(debug) << "duration param: " << diff.total_milliseconds();
// #endif
        }
    }
    
    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration pre-capture: " << diff.total_microseconds();
    if (!data && !sensor->capture(data)) {
        BOOST_LOG_TRIVIAL(warning) << "Could not capture from sensor.";
#ifdef BTA_P100
        //Issue in bta lib. We stop the application
        BOOST_LOG_TRIVIAL(warning)
                << "Camera not reacheable. Stopping the app.";
        exit(EXIT_FAILURE);
        /*
         BOOST_LOG_TRIVIAL(warning)
        << "Trying to reconnect....";
         if (sensor->reConnect() < 0) {
         BOOST_LOG_TRIVIAL(warning)
             << "Could not reconnect.";
         } else
         BOOST_LOG_TRIVIAL(info)
             << "Reconnected.";
         */
#endif
        //retries++;
        //if (retries > RETRY) {
        disconnect();
        //retries = 0;
        //}
        return false;
    }
    //retries = 0;
    //diff = boost::posix_time::microsec_clock::local_time() - start;
    //BOOST_LOG_TRIVIAL(info) << "duration read: " << diff.total_microseconds();
    /*
    // Checks if need to save raw data
    if ( (in.hasKey(name()+"save") || ((CapturerFilter*)this)->save()) &&
     !((CapturerFilter*)this)->playback() &&
     !bta_stream) {
    //Saving when flags on, and not playback, and not btastream which saves automatically
    cnt(cnt()+1);
    if ( in.hasKey(name()+"save") ) {
    //Old save from frame
        sensor->saveRaw(boost::any_cast<string>(in.getData(name()+"save")) +
            boost::lexical_cast<std::string>(cnt()) +
            RAWFILE,data);
        BOOST_LOG_TRIVIAL(debug) << "saved: " <<
                    boost::any_cast<string>(
                        in.getData(name()+"save")) +
                    boost::lexical_cast<std::string>(cnt()) +
                    RAWFILE;
    } else {
        sensor->saveRaw(strPath() + "/"
                + boost::lexical_cast<std::string>(cnt())
                +RAWFILE,data);
        BOOST_LOG_TRIVIAL(debug) << "saved: "
                     << strPath() + "/"
                    + boost::lexical_cast<std::string>(cnt())
                    +RAWFILE;
    }/ * else
        cnt(0);* /
    }
*/

    //BOOST_LOG_TRIVIAL(info) << "Start bta filter.";
    unsigned int mf,it;
    sensor->getFrameRef(data, mf, it);
    out.addData(_out_mf, mf);
    out.addData(_out_it, it );

    sensor->getFrameCounter(data, it);
    out.addData(_out_fc, it );
    out.addData("fc2", it );

    //The timestamp are unlogic.
    sensor->getFrameTime(data, it);
    out.addData(_out_ts, it );

    //Temps
    static float t;
    sensor->getMainTemp(data, t);
    out.addData(_out_mt, t );
    sensor->getLedTemp(data, t);
    out.addData(_out_lt, t );
    sensor->getGenericTemp(data, t);
    out.addData(_out_gt, t );

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration data: " << diff.total_microseconds();

    int size = distsSize, x=0,y=0;

    matPtr a, d;
    if ( ! out.hasKey(_out_depth) || size != distsSize) {
        // first iteration - initialize depth matrix
        sensor->getDisSize(data, x, y);

        width = x;
        height = y;
        distsSize = width*height;

        // initialize depth matrix ...:
        d.reset( new cv::Mat(height, width, CV_32F) );
        out.addData(_out_depth, d);

        a.reset( new cv::Mat(height, width, CV_16U) );

    } else {
        d = in.getMatPtr(_out_depth);
        a = in.getMatPtr(_out_ampl);
        x = width;
        y = height;
    }

    
    float* f = d->ptr<float>();
    sensor->getDistances( f ,distsSize,data);

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration depth: " << diff.total_microseconds();

    //sensor->getAmpSize(data, x, y);
    //unsigned short *amplitude= NULL;

    unsigned short* da = a->ptr<unsigned short>();
    sensor->getAmplitudes( da,size,data);

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration ampl: " << diff.total_microseconds();
    
    if (flip()){
        cv::flip(*a,*a,-1);
        cv::flip(*d,*d,-1);
    } else {
        if (flip_x()) {
            cv::flip(*a,*a,1);
            cv::flip(*d,*d,1);
        }
        if (flip_y()) {
            cv::flip(*a,*a,0);
            cv::flip(*d,*d,0);
        }
    }
    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration flip: " << diff.total_microseconds();

    out.addData(_out_depth, d );
    out.addData(_out_ampl, a );

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration add: " << diff.total_microseconds();

    sensor->freeFrame(data);
    data = NULL;

    //TODO MOVE TO OPENCV
    //if (!in.hasKey(name()+"2world") || update) {
    //    setCamera2Wcs(out,name()+"2world");
    //}

    diff = boost::posix_time::microsec_clock::local_time() - start;
    BOOST_LOG_TRIVIAL(debug) << "duration free: " << diff.total_microseconds();

    return true;
}

//TODO MOVE TO OPENCV
/*
void Bta::setCamera2Wcs(toffy::Frame& out, std::string name) {
    boost::shared_ptr<Eigen::Affine3f> f2t(new Eigen::Affine3f());

    //Rotations
    *f2t = pcl::getTransformation (0, 0, 0,
                   pcl::deg2rad(rotations().x),
                   pcl::deg2rad(rotations().y),
                   pcl::deg2rad(rotations().z));
    //Correct camera coord system --> 180? in z for argos
    // *f2t = pcl::getTransformation (0, 0, 0, 0, 0, pcl::deg2rad(180.)) * *f2t;
    // *f2t = Eigen::Scaling(1.f, 1.f, -1.f) * *f2t;
    //Translations
    *f2t = pcl::getTransformation (center_position().x,
                   center_position().y,
                   center_position().z, 0, 0, 0) * *f2t;

    out.addData(name,f2t);
}
*/

int Bta::connect() {

    int result = sensor->connect();
    if (result >= 0) {
        if (playback()) {
            float f = 0;
            int param = BTA_LibParamStreamPos;
            sensor->setLibParam(param, f);
            param = BTA_LibParamStreamTotalFrameCount;
            sensor->getLibParam(param, f);
            cout << "BTA_LibParamStreamTotalFrameCount: " << f << endl;
            endFile(f-1);
            beginFile(0);
            cnt(beginFile()-1);
        } else if (((CapturerFilter*)this)->save() && bta_stream) {
            BOOST_LOG_TRIVIAL(debug) << "strPath(): " << strPath();
            sensor->startGrabbing(strPath());
        }
    }
    return result;
}

int Bta::disconnect() {
    int result = sensor->disconnect();
    if (result > 0 && ((CapturerFilter*)this)->save() && bta_stream)
        sensor->stopGrabbing();
    return result;
}

bool Bta::isConnected() {
    return sensor->isConnected();
}

void Bta::playback(const bool &pb) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    BOOST_LOG_TRIVIAL(debug) << "bta_stream: " << bta_stream;
    BOOST_LOG_TRIVIAL(debug) << "pb: " << pb;
    if (pb == true && bta_stream) {
        sensor->setDeviceType(BTA_DeviceTypeGenericBltstream);
        /*if (sensor->getDeviceType() != BTA_DeviceTypeGenericBltstream) {
        BOOST_LOG_TRIVIAL(warning) << "Trying to read a bltstream file but the deviceType is not set to BTA_DeviceTypeGenericBltstream (15)!!!";
        return;
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

int Bta::loadPath(const std::string &newPath) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    fs::path fsPath(newPath);
    BOOST_LOG_TRIVIAL(debug) << "fsPath.extension(): " << fsPath.extension();
    if (fs::is_regular_file(fsPath) && fsPath.extension() == ".bltstream") {
        if ( !fs::exists(fsPath)) {
            BOOST_LOG_TRIVIAL(warning) << "Given path " << newPath
                                       << " does not exist.";
            return -1;
        }
        setLoadPath(newPath);
        if(isConnected()) {
            this->disconnect();
        }

        bta_stream = true;
        return 1;
    } else {
        bta_stream = false;
        return CapturerFilter::loadPath(newPath);
    }
}

void Bta::savePath(const std::string &newPath) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    fs::path fsPath(newPath);
    cout << "is regular: " << fs::is_regular_file(fsPath) << endl;
    if (fsPath.extension() == ".bltstream") {
        setSavePath(newPath);
        //bta_stream = true;
        return;
    } else {
        //bta_stream = false;
        return CapturerFilter::savePath(newPath);
    }
}

void Bta::save(const bool &save) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    if (save == true) {
        cout << "PATH::: " << getSavePath() << endl;
        long int _saveTimeStamp;
#if defined(MSVC)
        using namespace boost::posix_time;
        ptime pt(second_clock::local_time());
        static ptime epoch(boost::gregorian::date(1970, 1, 1));
        time_duration diff(pt - epoch);
        _saveTimeStamp= diff.ticks() / diff.ticks_per_second();
#else
        time(&_saveTimeStamp);
#endif
        //if (!getSavePath().empty() && fs::path(getSavePath()).extension() == ".bltstream") {
        //cout << "PATH::: " << saveFolder() << endl;
        fs::path newPath(getSavePath());
        //string _strPath = savePath + string("/") + id() + string("/") + boost::lexical_cast<std::string>(_saveTimeStamp);
        BOOST_LOG_TRIVIAL(debug) << newPath.string();
        if (tsd()) {
            newPath /= boost::lexical_cast<std::string>(_saveTimeStamp);
        }
        newPath /= name();
        BOOST_LOG_TRIVIAL(debug) << newPath.string();
        try {
            fs::create_directories(fs::absolute(newPath));
        } catch(const fs::filesystem_error& e) {
            BOOST_LOG_TRIVIAL(warning) << "Could not create folder: "
                                       << strPath() << "; Reason: "
                                       << e.code().message();
            setSave(false);
            return;
        }

        //newPath /= getSavePath();

        if (fs::path(getSavePath()).extension() != ".bltstream") {
            newPath /= boost::lexical_cast<std::string>(_saveTimeStamp) + ".bltstream";
        }

        strPath(newPath.string());

        cout << "PATH::: " << strPath() << endl;
        if (this->isConnected())
            sensor->startGrabbing(strPath());
        setSave(save);
    } else {
        if (this->isConnected())
            sensor->stopGrabbing();
        return CapturerFilter::save(save);
    }
}
