#ifdef MSVC
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <opencv2/opencv.hpp>
#if OPENCV_MAJOR >= 3
#include <opencv2/imgproc.hpp>
#else
#include <opencv2/imgproc/imgproc.hpp>
#endif

#if defined(ENABLE_PCL)
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl/conversions.h>
#endif

#include <boost/archive/basic_text_oarchive.hpp>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/bind.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/range/algorithm/remove_if.hpp>
#include <boost/range/algorithm_ext/erase.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/thread/thread.hpp>

#include <toffy/toffy_config.h>

#include <toffy/web/server/connection.hpp>
#include <toffy/web/server/mime_types.hpp>
#include <toffy/web/actions/action.hpp>

#include "toffy/web/requestController.hpp"
#include <toffy/web/requestAction.hpp>

#include <toffy/web/common/plugins.hpp>

//#include <toffy/web/btaController.hpp>
//#include <toffy/web/btaGroupController.hpp>
#include <toffy/web/capturersController.hpp>

#if defined(WITH_SENSOR2D)
#include <toffy/web/sensor2dController.hpp>
#include <toffy/web/sensorusbController.hpp>
#include <toffy/web/sensor2dGroupController.hpp>
#endif

// using namespace toffy;
using namespace toffy::control;
using namespace std;

/*Controller * Controller::_controller;


Controller * Controller::getInstance() {
    if(_controller == NULL) {
        _controller = new Controller();
    }
    return _controller;
}*/

Controller::~Controller() { BOOST_LOG_TRIVIAL(debug) << __FUNCTION__; }

int Controller::processRequest(http::server::connection_ptr con,
                               const http::server::request &req,
                               http::server::reply &rep)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    std::string &jsonReply = rep.content;
    Action action(req);
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    // TODO first should be l3vel, check in the grammar? needed?

    /* root actions (filterbank level)
     * /
     * /start
     * /stop
     * /loadConfig
     * /stepForward
     * /stepBackward
     */

    boost::property_tree::ptree jOutput;
    std::stringstream ss;
    string log;
    /* Vars:
     *
     * jsonReply -> string with the parser json as reply
     * jOutput -> Aux ptree to create the reply
     * log -> string param to methods that create the reply inside.
     */

    // If calling root of /toffy we return just ok. Used to check is
    // if server is on.
    /*if (action._items.size() <= 1) {
        jOutput.add("status","ok");
        boost::property_tree::json_parser::write_json(ss,jOutput);
        jsonReply = ss.str();
        return true;
    }*/

    std::string name, input;
    if (action._items.size() > 1) {
        name = action._items[1];
    }
    input = action.req->content;
    if (name == "loadConfig") {
        /*for (size_t i = 0; i < req.headers.size(); i++) {
            cout << req.headers[i].name << endl;
            cout << req.headers[i].value << endl;
        }*/
        // return false;
        if (!loadConfig(input, log)) {
            jOutput.add("status", "failed");
            jOutput.add("log", log);
            boost::property_tree::json_parser::write_json(ss, jOutput);
            log = ss.str();
            return false;
        }
        cout << "ois ok: " << log << endl;
    } else if (name == "image") {
        std::string name = action._items[2];
        int pos = action._items[2].rfind(".");
        if (pos > 0) {
            name = name.substr(0, pos);
        }
        BOOST_LOG_TRIVIAL(debug)
            << "get image " << name << " " << action._items[2];

        //      _thread = boost::thread( boost::bind(&Controller::loopImage,
        //      this, con,req,rep, name));

        return true;
#if defined(ENABLE_PCL)
    } else if (name == "cloud") {
        BOOST_LOG_TRIVIAL(debug) << "cloud";
        _thread = boost::thread(
            boost::bind(&Controller::loopCloud, this, con, req, rep));

        return true;
#endif
    } else if (name == "actions") {
        _thread = boost::thread(
            boost::bind(&Controller::loopAction, this, con, req, rep));

        return true;

    } else if (name == "start") {
        _c->forward();
    } else if (name == "stop") {
        if (!_c->stop()) {
            jOutput.add("status", "fail");
            jOutput.add("running", _c->getState());
            jOutput.add("reason", "Thread already stopped.");
            boost::property_tree::json_parser::write_json(ss, jOutput);
            log = ss.str();
            return false;
        }
    } else if (name == "forward") {
        BOOST_LOG_TRIVIAL(debug) << "found " << name;
        if (!_c->forward()) {
            jOutput.add("status", "fail");
            jOutput.add("running", _c->getState());
            jOutput.add("reason", "Thread already running. Stop it first.");
            boost::property_tree::json_parser::write_json(ss, jOutput);
            log = ss.str();
            return false;
        }
    } else if (name == "backward") {
        if (!_c->backward()) {
            jOutput.add("status", "fail");
            jOutput.add("running", _c->getState());
            jOutput.add("reason", "Thread already running. Stop it first.");
            boost::property_tree::json_parser::write_json(ss, jOutput);
        }
    } else if (name == "stepForward") {
        BOOST_LOG_TRIVIAL(debug) << "found " << name;
        if (!_c->stepForward()) {
            jOutput.add("status", "fail");
            jOutput.add("running", _c->getState());
            jOutput.add("reason", "Thread already running. Stop it first.");
            boost::property_tree::json_parser::write_json(ss, jOutput);
            log = ss.str();
            return false;
        }
    } else if (name == "stepBackward") {
        if (!_c->stedBackward()) {
            jOutput.add("status", "fail");
            jOutput.add("running", _c->getState());
            jOutput.add("reason", "Thread already running. Stop it first.");
            boost::property_tree::json_parser::write_json(ss, jOutput);
            log = ss.str();
            return false;
        }
    } else if (name == "filterlist") {
        boost::property_tree::ptree jsonList;

        this->collectFilters(log, _c->baseFilterBank, jOutput);
        jOutput.add_child("filters", jOutput);
        boost::property_tree::json_parser::write_json(ss, jOutput);
        jsonReply = ss.str();

    } else if (name == "filtertree") {
        boost::property_tree::ptree jsonList;

        this->collectFilterTree(log, _c->baseFilterBank, jsonList);
        jOutput.add_child("filters", jsonList);
        jsonReply = log;

    } else if (name == "filtergroup") {  // Finding a filter id
        std::vector<toffy::Filter *> vec;
        // By now only bta
        if (action._items[2] == "bta" || action._items[2] == "sensor2d") {
            _c->baseFilterBank->getFiltersByType(action._items[2], vec);
            CapturersController cgc(vec);
            cgc.doAction(action, log);
        } else if (action._items[2] == "capturers") {
            BOOST_LOG_TRIVIAL(debug)
                << "action._items[2]: " << action._items[2];
            _c->baseFilterBank->getFiltersByType("bta", vec);
            _c->baseFilterBank->getFiltersByType("sensor2d", vec);
            BOOST_LOG_TRIVIAL(debug) << "vec.size(): " << vec.size();
            CapturersController cgc(vec);
            cgc.doAction(action, log);
        } else {
            // should not happend
        }
        boost::property_tree::json_parser::write_json(ss, jOutput);
        log = ss.str();
        jsonReply = log;

    } else if (name == "configRuntime") {
        ss.clear();
        ss << input;
        boost::property_tree::ptree jsonValue;

        // TODO when this can fail?
        // std::cout << config << std::endl;
        boost::property_tree::json_parser::read_json(ss, jsonValue);
        if (jsonValue.empty()) {
            jOutput.add("status", "failed");
            jOutput.add("reason", "Json string wrong or empty;");
            ss.clear();
            boost::property_tree::json_parser::write_json(ss, jOutput);
            log = ss.str();
            return false;
        }
        boost::property_tree::ptree::const_assoc_iterator it =
            jsonValue.find("file");
        if (it == jsonValue.not_found()) {
            jOutput.add("status", "failed");
            jOutput.add("reason", "No key file found");
            ss.clear();
            boost::property_tree::json_parser::write_json(ss, jOutput);
            log = ss.str();
            return false;
        }

        if (action._items[2] == "save") {
            _c->saveRunConfig(it->second.data());
        } else if (action._items[2] == "load") {
            _c->loadRuntimeConfig(it->second.data());
        } else {
            jOutput.add("status", "failed");
            jOutput.add("log", "Unknown configRuntime action");
            return false;
        }

    } else if (!name.empty()) {
        // We assume /toffy/filter/id
        // We want the controller factory finding the filters and instantiating
        // the controller if exist
        Filter *f = _c->baseFilterBank->getFilter(name);
        // BOOST_LOG_TRIVIAL(debug) << f;
        // BOOST_LOG_TRIVIAL(debug) <<
        // _bankController.getFilter(action._items[1]);
        if (f == NULL) {
            jOutput.add("status", "failed");
            jOutput.add("reason", "Filter does not declared");
            boost::property_tree::json_parser::write_json(ss, jOutput);
            jsonReply = ss.str();
            return false;
        }

        BOOST_LOG_TRIVIAL(debug) << "Filter id: " << f->type();

        // if (f->type() == "bta") {
        /*    BtaController btaC(f);
            btaC.setBaseController(this);
            if (!btaC.doAction(action, jsonReply)) {
                return false;
            }*/
        //    return true;
#if defined(WITH_SENSOR2D)
    } else if (f->type() == "sensor2d") {
        Sensor2dController sensor2dC(f);
        if (!sensor2dC.doAction(action, jsonReply)) {
            return false;
        }
        return true;
    } else if (f->type() == "sensorUsb") {
        SensorUSBController sensor2dC(f);
        if (!sensor2dC.doAction(action, jsonReply)) {
            return false;
        }
        return true;
#endif
        //} else {
        FilterController *fc =
            ControllerFactory::getInstance()->getController(f);
        if (!fc->doAction(action, jsonReply)) {
            return false;
        }
        return true;
        //}
    }

    jOutput.add("status", "ok");
    jOutput.add("running", _c->getState());
    boost::property_tree::json_parser::write_json(ss, jOutput);
    rep.content = ss.str();
    return true;
}

void Controller::collectFilters(std::string &log, toffy::FilterBank *bank,
                                boost::property_tree::ptree &jsonList)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    std::cout << "Bank " << bank->name() << " " << bank->size() << std::endl;
    for (size_t i = 0; i < bank->size(); i++) {
        const Filter *f = bank->getFilter(i);
        // std::cout << "    filter " << f->name() << std::endl;
        log = "inspecting " + f->name();
        if (f->type() == "filterBank" || f->type() == "parallelFilter") {
            const FilterBank *fb = static_cast<const FilterBank *>(f);

            // jsonList.push_back(std::make_pair("",
            // bank->getFilter(i)->name()));
            jsonList.add("", bank->getFilter(i)->name());
            // jsonList.push_back(bank->getFilter(i)->name());
            collectFilters(log, (FilterBank *)fb, jsonList);
        } else {
            jsonList.add("", bank->getFilter(i)->name());
        }
    }
}

void Controller::collectFilterTree(std::string &log, toffy::FilterBank *bank,
                                   boost::property_tree::ptree &jsonList)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    for (size_t i = 0; i < bank->size(); i++) {
        const Filter *f = bank->getFilter(i);
        // std::cout << "    filter " << f->name() << std::endl;
        log = "inspecting " + f->name();
        if (f->type() == "filterBank" || f->type() == "parallelFilter") {
            const FilterBank *fb = static_cast<const FilterBank *>(f);

            boost::property_tree::ptree sublist;
            collectFilterTree(log, (FilterBank *)fb, sublist);
            jsonList.add_child(bank->getFilter(i)->name(), sublist);
        } else {
            jsonList.add(bank->getFilter(i)->id(), bank->getFilter(i)->name());
        }
    }
}

void Controller::loopImage(http::server::connection_ptr con,
                           const http::server::request &/*req*/,
                           http::server::reply &rep, std::string name)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    // http::server::reply rep;
    rep.headers.resize(7);
    rep.headers[0].name = "Max-Age";
    rep.headers[0].value = "0";
    rep.headers[1].name = "Content-Type";
    rep.headers[1].value =
        "multipart/x-mixed-replace;boundary=--BoundaryString";
    rep.headers[2].name = "Access-Control-Allow-Origin";
    rep.headers[2].value = "*";
    rep.headers[3].name = "Pragma";
    rep.headers[3].value = "no-cache";
    rep.headers[4].name = "Cache-Control";
    rep.headers[4].value = "no-cache, private";
    rep.headers[5].name = "Expires";
    rep.headers[5].value = "0";
    rep.headers[6].name = "Connection";
    rep.headers[6].value = "close";
    rep.status = http::server::reply::ok;

    rep.content.empty();
    std::vector<boost::asio::const_buffer> buf = rep.to_buffers();
    con->socket().send(buf);

    // imgencode params:
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(75);  // 0..100 ; 100=high quality

    BOOST_LOG_TRIVIAL(debug) << "START THREAD FOR " << name;

    using namespace cv;

    cv::Mat body;
    cv::Mat show;

    body = cv::Mat::zeros(cv::Size(160, 120), CV_8U);

    while (con->socket().is_open()) {
        BOOST_LOG_TRIVIAL(debug)
            << "IMAGE THREAD " << con->socket().available();

        if (_c->f.hasKey(name)) {
            try {
                body = _c->f.getMatPtr(name)->clone();
            } catch (std::exception &e) {
                BOOST_LOG_TRIVIAL(debug)
                    << "Todo: fix race condition " << e.what();
                boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                continue;
            }

            if (body.type() == CV_8UC3) {
                // nop op
                show = body;
            } else {
                body.convertTo(body, CV_8UC1, 256, 0);
                cv::cvtColor(body, show, cv::COLOR_GRAY2RGB);
            }
            BOOST_LOG_TRIVIAL(debug)
                << "img loop had " << body.type() << " " << show.size();

            cv::flip(show, show, 1);
            cv::resize(show, show, cv::Size(), 2, 2);

        } else {
            BOOST_LOG_TRIVIAL(debug) << "img loop didn't find " << name;

            show = cv::Mat::zeros(cv::Size(320, 240), CV_8UC3);

            putText(show, name + " not found...", cv::Point(12, 12),
                    FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
        }

        std::vector<uchar> outbuf(show.cols * show.rows);

        cv::imencode("hand.jpg", show, outbuf, params);

        const std::string mpartHeader =
            "--BoundaryString\r\n"
            "Content-type: image/jpeg\r\n"
            "Content-Length: " +
            boost::lexical_cast<std::string>(outbuf.size()) + "\r\n\r\n";
        try {
            con->socket().send(boost::asio::buffer(mpartHeader));
            con->socket().send(boost::asio::buffer(outbuf));
        } catch (std::exception &e) {
            BOOST_LOG_TRIVIAL(debug) << "Problem sending data: " << e.what();
            break;
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        BOOST_LOG_TRIVIAL(debug) << "IMAGE THREAD LOOP";
    }
    BOOST_LOG_TRIVIAL(debug) << "Thread " << __FUNCTION__ << " ends";
}

bool Controller::loadConfig(std::string config, std::string &log)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    std::stringstream ss(config);
    boost::property_tree::ptree jsonValue, jOut;
    boost::property_tree::ptree pxml;
    try {
        boost::property_tree::read_xml(ss, pxml);
        if (_c->loadConfig(pxml) < 1) {
            jOut.add("status", "failed");
            jOut.add("reason", "Wrong xml file");
            ss.str(std::string());
            boost::property_tree::json_parser::write_json(ss, jOut);
            log = ss.str();
            return false;
        } else {
            // TODO LOAD WEB UI controllers if exist.
            std::vector<void *> loads = _c->getLoadedFilters();
            for (size_t i = 0; i < loads.size(); i++) {
#ifdef MSVC
                toffy::commons::plugins::initUI_t initUI =
                    (toffy::commons::plugins::initUI_t)GetProcAddress(
                        static_cast<HINSTANCE>(loads[i]), "initUI");
                if (!initUI) {
                    BOOST_LOG_TRIVIAL(warning)
                        << "Could not load plug-in filters interface.";
#else
                toffy::commons::plugins::initUI_t initUI =
                    (toffy::commons::plugins::initUI_t)dlsym(loads[i],
                                                             "initUI");
                const char *dlsym_error = dlerror();
                if (dlsym_error) {
                    BOOST_LOG_TRIVIAL(info)
                        << "Could not load plug-in filters interface: "
                        << std::endl
                        << dlsym_error;
#endif
                } else {
                    // use it to do the calculation
                    BOOST_LOG_TRIVIAL(info)
                        << "Loaded plug-in filters interface";
                    // use it to do the calculation
                    // std::cout << "Calling hello...\n";
                    initUI(ControllerFactory::getInstance());
                }
            }
            // pxm = _c->_loads

            jOut.add("status", "ok");
            ss.str(std::string());
            boost::property_tree::json_parser::write_json(ss, jOut);
            log = ss.str();
            return true;
        }
    } catch (boost::property_tree::xml_parser_error &e) {
        BOOST_LOG_TRIVIAL(warning) << "Input data not xml. Try json";
    }
    // TODO when this can fail?
    boost::property_tree::json_parser::read_json(ss, jsonValue);
    if (jsonValue.empty()) {
        jOut.add("status", "failed");
        jOut.add("reason", "Json string wrong or empty;");
        ss.clear();
        boost::property_tree::json_parser::write_json(ss, jOut);
        log = ss.str();
        return false;
    }

    // TODO we do not want values as root

    boost::property_tree::ptree::const_assoc_iterator it =
        jsonValue.find("file");
    if (it != jsonValue.not_found()) {
        _c->baseFilterBank->loadFileConfig(it->second.data());
        jOut.add("status", "ok");
    } else {
        jOut.add("status", "failed");
        jOut.add("reason", "No key file found");
    }
    ss.clear();
    boost::property_tree::json_parser::write_json(ss, jOut);
    log = ss.str();
    return true;
    // TODO what should be readed in here?????
}

const std::string base64_padding[] = {"", "==", "="};
std::string base64EncodeData(std::vector<uint8_t> data)
{
    using namespace boost::archive::iterators;
    typedef std::vector<uint8_t>::const_iterator iterator_type;
    typedef base64_from_binary<transform_width<iterator_type, 6, 8> >
        base64_enc;
    std::stringstream ss;
    std::copy(base64_enc(data.begin()), base64_enc(data.end()),
              ostream_iterator<char>(ss));
    ss << base64_padding[data.size() % 3];
    return ss.str();
}

void Controller::loopCloud(http::server::connection_ptr con,
                           const http::server::request &req,
                           http::server::reply &rep)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
#if defined(ENABLE_PCL)
    rep.headers.resize(3);
    rep.headers[0].name = "Content-Length";
    // rep.headers[0].value =
    // boost::lexical_cast<std::string>(rep.content.size());
    rep.headers[1].name = "Content-Type";
    rep.headers[1].value = http::server::mime_types::extension_to_type("json");
    rep.headers[2].name = "Access-Control-Allow-Origin";
    rep.headers[2].value = "*";

    BOOST_LOG_TRIVIAL(debug) << "START THREAD";

    rep.status = http::server::reply::ok;

    pcl::PCLPointCloud2 pc2 =
        *boost::any_cast<pcl::PCLPointCloud2Ptr>(_c->f.getData("cloud"));

    pcl::PointCloud<pcl::PointXYZ> xyz;

    pcl::fromPCLPointCloud2(pc2, xyz);
    pcl::toPCLPointCloud2(xyz, pc2);

    std::stringstream ss;

    boost::property_tree::ptree jOutput;

    jOutput.add("cloud.header.frame_id", pc2.header.frame_id);
    jOutput.add("cloud.header.seq", pc2.header.seq);
    jOutput.add("cloud.header.stamp", pc2.header.stamp);

    for (size_t i = 0; i < pc2.fields.size(); i++) {
        std::string cnt = boost::lexical_cast<std::string>(i);
        jOutput.add("cloud.fields." + cnt + ".name", pc2.fields[i].name);
        jOutput.add("cloud.fields." + cnt + ".offset", pc2.fields[i].offset);
        jOutput.add("cloud.fields." + cnt + ".count", pc2.fields[i].count);
        jOutput.add("cloud.fields." + cnt + ".datatype",
                    pc2.fields[i].datatype);
    }

    jOutput.add("cloud.height", pc2.height);
    jOutput.add("cloud.width", pc2.width);

    jOutput.add("cloud.is_bigendian", pc2.is_bigendian);
    jOutput.add("cloud.point_step", pc2.point_step);
    jOutput.add("cloud.row_step", pc2.row_step);

    using namespace boost::archive::iterators;
    typedef std::vector<pcl::uint8_t>::const_iterator iterator_type;
    typedef base64_from_binary<transform_width<iterator_type, 6, 8> >
        base64_enc;
    std::copy(base64_enc(pc2.data.begin()), base64_enc(pc2.data.end()),
              ostream_iterator<pcl::uint8_t>(ss));

    // std::string s = ss.str();
    // std::string s = base64EncodeData(pc2.data);

    jOutput.add("cloud.data", ss.str());

    jOutput.add("cloud.is_dense", pc2.is_dense);

    jOutput.add("status", "ok");
    // jOutput.add("reason", "Json string wrong or empty;");
    ss.str(std::string());
    boost::property_tree::json_parser::write_json(ss, jOutput, false);
    rep.content = ss.str();

    rep.headers[0].value = boost::lexical_cast<std::string>(rep.content.size());
    /*std::vector<uchar>outbuf;

    rep.content =  "--BoundaryString\r\n"
        + std::string("Content-type: image/jpg\r\n")
        + std::string("Content-Length: ")
        + boost::lexical_cast<std::string>(outbuf.size())
        + "\r\n\r\n"
        + std::string(outbuf.begin(),outbuf.end());*/

    boost::asio::async_write(
        con->socket(), rep.to_buffers(),
        boost::bind(&http::server::connection::handle_write /*_nostop*/, con,
                    boost::asio::placeholders::error));
    // usleep(500);
    // BOOST_LOG_TRIVIAL(debug) << "IMAGE THREAD LOOP";
    //}
    cout << "THREAD ends: " << endl;
    // exit(0);
#endif
    BOOST_LOG_TRIVIAL(debug) << "Thread " << __FUNCTION__ << " ends";
}

void Controller::loopAction(http::server::connection_ptr con,
                            const http::server::request &/*req*/,
                            http::server::reply &rep)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    rep.headers.resize(3);
    rep.headers[0].name = "Content-Length";
    // rep.headers[0].value =
    // boost::lexical_cast<std::string>(rep.content.size());
    rep.headers[1].name = "Content-Type";
    rep.headers[1].value = http::server::mime_types::extension_to_type("json");
    rep.headers[2].name = "Access-Control-Allow-Origin";
    rep.headers[2].value = "*";
    rep.status = http::server::reply::ok;

    BOOST_LOG_TRIVIAL(debug) << "START THREAD";

    cout << "START THREAD" << endl;

    std::stringstream ss;

    boost::property_tree::ptree jOutput;

    // TODO block, no return
    boost::shared_ptr<
        boost::circular_buffer<boost::shared_ptr<toffy::Actions::Action> > >
        actionsBuffer;
    try {
        actionsBuffer =
            boost::any_cast<boost::shared_ptr<boost::circular_buffer<
                boost::shared_ptr<toffy::Actions::Action> > > >(
                _c->f.getData("actions"));
    } catch (const boost::bad_any_cast &) {
        jOutput.add("status", "error");
        jOutput.add("reason", "No actions");
        ss.str(std::string());
        boost::property_tree::json_parser::write_json(ss, jOutput, false);
        rep.content = ss.str();
        cout << "rep.content: " << rep.content << endl;
        cout << "rep.content: " << rep.content.size() << endl;
        rep.headers[0].value =
            boost::lexical_cast<std::string>(rep.content.size());
        cout << "rep.headers[0].value: " << rep.headers[0].value << endl;
        boost::asio::async_write(
            con->socket(), rep.to_buffers(),
            boost::bind(&http::server::connection::handle_write /*_nostop*/,
                        con, boost::asio::placeholders::error));
        cout << "THREAD ends no actions " << endl;
        return;
    }
    cout << "Found actions!! " << endl;
    cout << "actionsBuffer: " << actionsBuffer->size() << endl;
    if (actionsBuffer->empty()) {
        jOutput.add("status", "error");
        jOutput.add("reason", "Actions Empty");
        ss.str(std::string());
        boost::property_tree::json_parser::write_json(ss, jOutput, false);
        rep.content = ss.str();
        cout << "rep.content: " << rep.content << endl;
        cout << "rep.content: " << rep.content.size() << endl;
        rep.headers[0].value =
            boost::lexical_cast<std::string>(rep.content.size());
        cout << "rep.headers[0].value: " << rep.headers[0].value << endl;
        boost::asio::async_write(
            con->socket(), rep.to_buffers(),
            boost::bind(&http::server::connection::handle_write /*_nostop*/,
                        con, boost::asio::placeholders::error));
        cout << "THREAD ends no actions " << endl;
        return;
    }

    actionsBuffer->front()->toJsonString(jOutput);
    actionsBuffer->pop_front();

    jOutput.add("status", "ok");

    ss.str(std::string());
    boost::property_tree::json_parser::write_json(ss, jOutput, false);
    rep.content = ss.str();

    rep.headers[0].value = boost::lexical_cast<std::string>(rep.content.size());

    boost::asio::async_write(
        con->socket(), rep.to_buffers(),
        boost::bind(&http::server::connection::handle_write /*_nostop*/, con,
                    boost::asio::placeholders::error));

    cout << "THREAD ends: " << endl;
    // exit(0);
    BOOST_LOG_TRIVIAL(debug) << "Thread " << __FUNCTION__ << " ends";
}
