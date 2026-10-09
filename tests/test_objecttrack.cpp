/*
   Copyright 2026 Simon Vogl <svogl@voxel.at>

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

// Fence for N4 / P3-5 in ObjectTrack.
//
// ObjectTrack::publishCount() handed a configuration string to a shell:
//
//   std::stringstream sb;
//   sb << updateScript << " " << cnt << " " << numObjects << "&";
//   system(sb.str().c_str());
//
// `updateScript` is `options/script`, read straight out of XML in
// updateConfig(). Any shell metacharacter in a config file therefore ran with
// this process's privileges, and the trailing `&` meant the call did not even
// wait for it. GCC had been reporting the ignored return value of system() on
// every build (-Wunused-result).
//
// The fix runs the script with execvp() on an argv array, so the configured
// value is a program name and its arguments are arguments. These tests pin both
// halves: that the script still runs and still receives the count, and that a
// config file cannot run anything else. The second one is the fence and was
// observed creating its marker file on the parent commit.
//
// The filter is driven through filter() rather than at publishCount() directly,
// because publishCount() is private and staying private is the point: the
// injection was reachable through the documented config path, so that is what
// the test goes through. One DetectedObject with an empty `record` is enough to
// reach the publish at the end of filter() while skipping the per-object body,
// which ends in cv::namedWindow()/imshow() and would need a display.

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <toffy/detection/detectedObject.hpp>
#include <toffy/detection/objectTrack.hpp>
#include <toffy/frame.hpp>

namespace {

using toffy::detection::DetectedObjects;
using toffy::detection::DetectedObject;

/** An <objectTrack> node with only the script option set. */
boost::property_tree::ptree config(const std::string& script) {
    boost::property_tree::ptree node;
    node.put("options.script", script);
    return node;
}

/** A fresh directory per test, so file existence is unambiguous. */
std::string scratchDir(const std::string& name) {
    const std::string dir = ::testing::TempDir() + "/objecttrack_" + name;
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir);
    return dir;
}

/** Write an executable shell script and make it runnable (0755). */
void writeExecutable(const std::string& path, const std::string& body) {
    std::ofstream f(path);
    f << body;
    f.close();
    std::filesystem::permissions(path,
                                 std::filesystem::perms::owner_all |
                                     std::filesystem::perms::group_read |
                                     std::filesystem::perms::group_exec |
                                     std::filesystem::perms::others_read |
                                     std::filesystem::perms::others_exec);
}

std::string readFile(const std::string& path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/**
 * Poll until `path` exists and has content, or `ms` elapses.
 *
 * The existence test has to be separate: `file_size()` on a missing file sets
 * `ec` and returns the maximum of `uintmax_t`, which is > 0, so a size check
 * alone reports "the script has written" the instant it is asked. Observed:
 * with only the size check this helper returned true immediately, the first
 * test read an empty file and the last one removed a file that had not been
 * created yet, so its async write landed during the sleep that was meant to
 * catch a second publish.
 */
bool waitForNonEmptyFile(const std::string& path, int ms) {
    std::error_code ec;
    for (int waited = 0; waited < ms; waited += 20) {
        if (std::filesystem::exists(path) &&
            std::filesystem::file_size(path, ec) > 0) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

/**
 * A frame with a small depth image and `n` detected objects.
 *
 * The objects have an empty `record`, which is what `filter()` treats as
 * "nothing to report about this object" and skips; the publish at the end of
 * the filter runs on the object *count* alone. Objects are owned by the caller,
 * as the vector holds raw pointers.
 */
class Scene {
public:
    Scene(std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) {
            DetectedObject* o = new DetectedObject();
            o->record = std::make_shared<
                boost::circular_buffer<DetectedObject*>>();
            _objects.push_back(o);
        }
    }

    ~Scene() {
        for (std::size_t i = 0; i < _objects.size(); ++i) delete _objects[i];
    }

    /**
     * Put the scene in `frame`. The vector the frame ends up holding contains
     * raw pointers, so this Scene must outlive every filter() call made on it:
     * the objects are deleted here, not by the frame.
     */
    void addTo(toffy::Frame& frame) const {
        std::shared_ptr<DetectedObjects> objs(new DetectedObjects(_objects));
        frame.addData("objects", objs);
        frame.addData("img", toffy::matPtr(
                                new cv::Mat(8, 8, CV_8UC1, cv::Scalar(7))));
    }

private:
    DetectedObjects _objects;
};

}  // namespace

TEST(ObjectTrackPublishCount, ScriptRunsAndReceivesTheCountAsArguments)
{
    const std::string dir = scratchDir("runs");
    const std::string out = dir + "/argv.txt";
    writeExecutable(dir + "/count.sh",
                    "#!/bin/sh\necho \"$1 $2\" >> " + out + "\n");

    toffy::detection::ObjectTrack tracker;
    tracker.updateConfig(config(dir + "/count.sh"));

    Scene scene(1);
    toffy::Frame frame;
    scene.addTo(frame);
    EXPECT_TRUE(tracker.filter(frame, frame));

    ASSERT_TRUE(waitForNonEmptyFile(out, 2000))
        << "the configured script never ran";

    // cnt is 0 (no object crossed a zone) and the object count is the one from
    // the previous frame: publishCount() runs before `numObjects` is updated,
    // so the first publish reports 0 objects while there is one. Pinned here as
    // it behaves, filed as A27; whoever fixes it should expect this to go red.
    EXPECT_EQ("0 0\n", readFile(out));
}

TEST(ObjectTrackPublishCount, ShellMetacharactersInScriptAreNotInterpreted)
{
    // Each payload is one program name that a shell would read as several
    // commands. Pre-fix every one of them was concatenated into a command line
    // and the second command ran: observed on the parent commit, where `pwned`
    // appeared for all six and count.sh had been executed with no arguments.
    const char* const kSuffixes[] = {
        "; touch PN", "&& touch PN", "| touch PN", "$(touch PN)",
        "`touch PN`", "\ntouch PN"};

    for (std::size_t p = 0; p < sizeof(kSuffixes) / sizeof(kSuffixes[0]); ++p) {
        const std::string suffix = kSuffixes[p];
        // A fresh directory per payload: the previous payload's shell must not
        // be able to satisfy or spoil this one's assertions.
        const std::string dir = scratchDir("meta_" + std::to_string(p));
        const std::string argv = dir + "/argv.txt";
        const std::string pwned = dir + "/pwned";
        writeExecutable(dir + "/count.sh",
                        "#!/bin/sh\necho ran >> " + argv + "\n");
        std::string payload = dir + "/count.sh" + suffix;
        // "PN" stands for the marker path in the table above, so the payloads
        // stay readable; the marker is the same file either way.
        for (std::size_t at = payload.find("PN"); at != std::string::npos;
             at = payload.find("PN", at)) {
            payload.replace(at, 2, pwned);
        }

        SCOPED_TRACE("payload: " + payload);

        toffy::detection::ObjectTrack tracker;
        tracker.updateConfig(config(payload));

        Scene scene(1);
        toffy::Frame frame;
        scene.addTo(frame);
        EXPECT_TRUE(tracker.filter(frame, frame));

        // Give a pre-fix background shell every chance to be scheduled.
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        EXPECT_FALSE(std::filesystem::exists(pwned))
            << "a configuration file ran a command through a shell";
        EXPECT_FALSE(std::filesystem::exists(argv))
            << "the part before the metacharacter was executed anyway";
    }
}

TEST(ObjectTrackPublishCount, AProgramThatCannotBeExecutedIsNotAnErrorForTheBank)
{
    const std::string dir = scratchDir("missing");
    toffy::detection::ObjectTrack tracker;
    // The default `updateCount.sh` is a relative name that no configuration
    // provides, so this is the path every run without the script takes. A
    // filter returning false aborts the whole bank, so an unrunnable helper has
    // to be reported and survived, not fatal.
    tracker.updateConfig(config(dir + "/does-not-exist.sh"));

    Scene one(1);
    toffy::Frame frame;
    one.addTo(frame);
    EXPECT_TRUE(tracker.filter(frame, frame));

    // Still usable afterwards, and still publishing on change.
    Scene two(2);
    toffy::Frame second;
    two.addTo(second);
    EXPECT_TRUE(tracker.filter(second, second));
}

TEST(ObjectTrackPublishCount, NothingIsPublishedWhileTheObjectCountIsStable)
{
    const std::string dir = scratchDir("stable");
    const std::string argv = dir + "/argv.txt";
    writeExecutable(dir + "/count.sh",
                    "#!/bin/sh\necho ran >> " + argv + "\n");

    toffy::detection::ObjectTrack tracker;
    tracker.updateConfig(config(dir + "/count.sh"));

    Scene firstScene(1);
    toffy::Frame first;
    firstScene.addTo(first);
    EXPECT_TRUE(tracker.filter(first, first));
    ASSERT_TRUE(waitForNonEmptyFile(argv, 2000));

    std::filesystem::remove(argv);

    // Same number of objects: no change, so no script. This is the property
    // that made the old fire-and-forget shell call tempting, and the reason the
    // replacement must not block on the script either.
    Scene againScene(1);
    toffy::Frame again;
    againScene.addTo(again);
    EXPECT_TRUE(tracker.filter(again, again));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_FALSE(std::filesystem::exists(argv));
}
