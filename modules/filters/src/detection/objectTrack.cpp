/*
   Copyright 2018 Simon Vogl <svogl@voxel.at>
                  Angel Merino-Sastre <amerino@voxel.at>

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
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>

// POSIX only, like the rest of toffy (P2-2): the process primitives publishCount
// needs are fork/execvp/waitpid, and there is no Windows variant to keep in step.
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#include <opencv2/highgui.hpp>

#include <toffy/detection/detectedObject.hpp>
#include <toffy/detection/objectTrack.hpp>

using namespace toffy;
using namespace toffy::detection;
// using namespace cv;
using namespace std;

#ifdef CM_DEBUG
static const bool dbg = true;
#else
static const bool dbg = false;
#endif

std::size_t ObjectTrack::_filter_counter = 1;
const std::string ObjectTrack::id_name = "objectTrack";

ObjectTrack::ObjectTrack()
    : Filter(ObjectTrack::id_name, _filter_counter),
      _in_img("img"),
      _in_vec("objects"),
      _out_vec("dect"),
      cnt(0),
      numObjects(0),
      updateScript("updateCount.sh")
{
    _filter_counter++;
}

boost::property_tree::ptree ObjectTrack::getConfig() const
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();
    // pt.put("options.max_x", _max_x);

    pt.put("inputs.img", _in_img);
    pt.put("inputs.vec", _in_vec);
    pt.put("outputs.vec", _out_vec);

    return pt;
}

void ObjectTrack::updateConfig(const boost::property_tree::ptree& pt)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    //_max_x = pt.get<float>("options.max_x",_max_x);

    _in_img = pt.get<string>("inputs.img", _in_img);
    _in_vec = pt.get<string>("inputs.vec", _in_vec);
    _out_vec = pt.get<string>("outputs.vec", _out_vec);

    updateScript = pt.get<string>("options.script", updateScript);
}

bool ObjectTrack::filter(const Frame& in, Frame& out)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();

    toffy::Filter::setLoggingLvl();

    // List of detected objects
    BOOST_LOG_TRIVIAL(debug) << "Getting _in_vec: " << _in_vec;
    std::shared_ptr<DetectedObjects> detObjs;
    try {
        detObjs = boost::any_cast<std::shared_ptr<DetectedObjects> >(
            in.getData(_in_vec));
    } catch (const boost::bad_any_cast&) {
        BOOST_LOG_TRIVIAL(warning)
            << "Could not find object vector: " << _in_vec << ". " << id();
        return true;
    }

    BOOST_LOG_TRIVIAL(debug)
        << "Got _in_vec(" << _in_vec << "): " << detObjs->size();

    // TODO Where come img from?, why we mask it with the objects.
    BOOST_LOG_TRIVIAL(debug) << "Getting got _in_img: " << _in_img;
    matPtr img;
    try {
        img = in.getMatPtr(_in_img);
    } catch (const boost::bad_any_cast&) {
        BOOST_LOG_TRIVIAL(warning)
            << "Could not cast input " << _in_img << ", filter  " << id()
            << " does not show objects.";
    }

    // Define zones!!!
    cv::Rect z1(0, 0, img->cols, (img->rows / 2) - 1);

    cv::Rect z2(0, 0, img->cols, (img->rows / 2) - 1);

    cv::Mat color;

    double max, min;
    minMaxIdx(*img, &min, &max);
    img->convertTo(color, CV_8U, 255.0 / (max - min),
                   -255.0 * min / (max - min));
    cvtColor(color, color, cv::COLOR_GRAY2RGB);

    vector<detection::DetectedObject*>::iterator objDetIter = detObjs->begin();
    while (objDetIter != detObjs->end()) {
        detection::DetectedObject* obj = *objDetIter;
        if (obj->record->empty()) {
            objDetIter++;
            continue;
        }

        if (z1.contains(obj->firstCenter)) {
            if (z1.contains(obj->massCenter))
                BOOST_LOG_TRIVIAL(debug) << "IN ZONE 1";
            else {
                obj->firstCenter = obj->massCenter;
                BOOST_LOG_TRIVIAL(debug) << "Change to zone 2";
                cnt++;
                publishCount();
            }
        } else {
            if (!z1.contains(obj->massCenter))
                BOOST_LOG_TRIVIAL(debug) << "IN ZONE 2";
            else {
                obj->firstCenter = obj->massCenter;
                BOOST_LOG_TRIVIAL(debug) << "Change to zone 1";
                cnt--;
                publishCount();
            }
        }

        BOOST_LOG_TRIVIAL(debug)
            << "Old: " << obj->record->back()->massCenter3D;
        BOOST_LOG_TRIVIAL(debug) << "New: " << obj->massCenter3D;
        double dis =
            cv::norm(obj->massCenter3D - obj->record->front()->massCenter3D);
        BOOST_LOG_TRIVIAL(debug) << "dis: " << dis;
        boost::posix_time::time_duration time =
            obj->record->front()->ts - obj->ts;
        BOOST_LOG_TRIVIAL(debug) << "time: " << time.seconds();
        double speed = dis / time.seconds();
        BOOST_LOG_TRIVIAL(debug) << "speed: " << speed;

        cv::arrowedLine(color, obj->record->back()->massCenter, obj->massCenter,
                        CV_RGB(255, 255, 0));

        cv::line(color, cv::Point(z1.x, z1.height), z1.br(), CV_RGB(255, 0, 0),
                 1);

        cv::putText(color, boost::str(boost::format("%.4f") % speed) + " m/s",
                    cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    CV_RGB(255, 255, 0));

        cv::putText(color, boost::lexical_cast<std::string>(cnt),
                    cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    CV_RGB(255, 255, 0));

        objDetIter++;

        cv::namedWindow("objectTrack", cv::WINDOW_NORMAL);
        imshow("objectTrack", color);
    }
    if (numObjects != detObjs->size()) publishCount();

    numObjects = detObjs->size();

    return true;
}

namespace {

/**
 * Run `program a1 a2` in the background, with no shell anywhere in the path.
 *
 * This is finding N4 / `P3-5`. publishCount() used to do
 *
 *     sb << updateScript << " " << cnt << " " << numObjects << "&";
 *     system(sb.str().c_str());
 *
 * and `updateScript` is `options/script`, read straight out of XML. A config
 * file containing "count.sh; rm -rf ..." therefore ran the second command with
 * this process's privileges, and the "&" was inside the string, so /bin/sh
 * backgrounded the script and exited: system() waited for the shell, never for
 * the script. tests/test_objecttrack.cpp pinned that before the fix - a config
 * value containing a ";" created a file it named.
 *
 * What was load-bearing about the old call was that the frame loop does not
 * block on a user script and that the script's exit status is nobody's business.
 * Both are kept. What is dropped is the parsing: the program and its arguments
 * go to execvp() as an argv array, so a metacharacter in a configuration file is
 * a character in a file name and nothing more.
 *
 * The double fork is what replaces the "&". The intermediate child exits
 * immediately, so the script is reparented to init and reaped by init instead of
 * becoming a zombie of ours on every zone change; waiting for the *intermediate*
 * is what stops this function leaking a zombie of its own, and it returns as
 * soon as the intermediate is done, which is before the script's exec has been
 * reached. The wait is as bounded as system()'s ever was: it can only outlast
 * this call if the intermediate child is stopped by a debugger or a SIGSTOP.
 *
 * Between fork() and exec only async-signal-safe functions may be called, so the
 * child does exactly one thing and reports failure with write() and _exit() -
 * not with BOOST_LOG, whose locks another thread may be holding at fork() time.
 */
void runDetached(const std::string &program, const std::string &a1,
                 const std::string &a2)
{
    // A clean diagnostic is only possible for a path: a bare name is resolved
    // against $PATH by execvp(), and duplicating that search here to check it
    // would be a second implementation to keep honest. A bare name that cannot
    // be executed is therefore reported by the child, on stderr, exactly as the
    // shell used to report it.
    if (program.find('/') != std::string::npos &&
        access(program.c_str(), X_OK) != 0) {
        // Reported, not thrown: a filter returning false aborts the whole bank,
        // and an unrunnable helper is not that. Every configuration that never
        // set options/script lands here too, because the default is the relative
        // name "updateCount.sh" - the shell printed the same complaint on stderr
        // once per call, so this is not a new message, only a filterable one.
        BOOST_LOG_TRIVIAL(warning)
            << "publishCount: cannot execute " << program << ": "
            << strerror(errno);
        return;
    }

    char *const argv[] = {const_cast<char *>(program.c_str()),
                          const_cast<char *>(a1.c_str()),
                          const_cast<char *>(a2.c_str()), nullptr};

    const pid_t intermediate = fork();
    if (intermediate < 0) {
        BOOST_LOG_TRIVIAL(warning)
            << "publishCount: fork failed: " << strerror(errno);
        return;
    }

    if (intermediate == 0) {
        const pid_t script = fork();
        if (script > 0) {
            // The script outlives this child and belongs to init from here on.
            _exit(0);
        }
        if (script == 0) {
            execvp(program.c_str(), argv);
            // Only reached if exec failed, which the access() above makes
            // unlikely for a path: the file can still have gone away since.
            const std::string msg =
                "toffy: cannot execute " + program + ": " + strerror(errno) +
                "\n";
            const ssize_t ignored =
                write(STDERR_FILENO, msg.data(), msg.size());
            (void)ignored;
            _exit(127);
        }
        _exit(127);
    }

    int status = 0;
    while (waitpid(intermediate, &status, 0) == -1 && errno == EINTR) {
        // A signal that interrupted the wait is not a reason to leak the child.
    }
}

}  // namespace

void ObjectTrack::publishCount()
{
    // The two numbers the helper script is given: the zone counter and the
    // number of objects. numObjects is still the *previous* frame's count here,
    // because filter() publishes before it updates the member. Pinned as it
    // behaves by tests/test_objecttrack.cpp and filed as A27; fixing it is a
    // behaviour change and not this item's.
    const std::string count = boost::lexical_cast<std::string>(cnt);
    const std::string objects = boost::lexical_cast<std::string>(numObjects);

    BOOST_LOG_TRIVIAL(debug) << "publ: " << updateScript << " " << count << " "
                             << objects;

    runDetached(updateScript, count, objects);
}
