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
#include <boost/foreach.hpp>
#include <boost/log/trivial.hpp>

#include <opencv2/core.hpp>

#include <toffy/common/filenodehelper.hpp>
#include <toffy/filter_helpers.hpp>

#include "toffy/import/importYaml.hpp"

using namespace toffy;
using namespace toffy::commons;
using namespace toffy::import;
using namespace cv;
using namespace std;

std::size_t ImportYaml::_filter_counter = 1;

ImportYaml::ImportYaml() : Filter("importYaml", _filter_counter)
{
    _filter_counter++;
}

boost::property_tree::ptree ImportYaml::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.path", path);
    pt.put("options.prefix", prefix);

    pt.put("options.start", start);

    return pt;
}

void ImportYaml::updateConfig(const boost::property_tree::ptree& pt)
{
    LOG(debug) << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    _seq = pt.get<bool>("options.sequence", _seq);

    path = pt.get("options.path", path);
    prefix = pt.get("options.prefix", path);
    start = pt.get("options.start", start);
    counter = start;
}

bool ImportYaml::filter(const Frame& in, Frame& out)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__ << " " << id();
    UNUSED(in);
    char buf[80] = "";

    snprintf(buf, sizeof(buf), "%04d", counter);

    bool success = loadFromYaml(out, std::string(buf));
    if (!success) {
        if (counter == start) {
            return false;  // none ever worked.. complain!
        }
        // recursive retry.. lisp danger ahead :)
        BOOST_LOG_TRIVIAL(info) << id() << "RESTARTING AT " << counter;
        counter = start;
        return filter(in, out);
    }
    return success;
}

static inline void putMat(Frame& f, cv::FileStorage& file,
                          const std::string& slot)
{
    if (!file[slot].isNone()) {
        matPtr m;
        if (f.hasKey(slot)) {
            m = f.getMatPtr(slot);
        } else {
            m.reset(new Mat());
        }
        file[slot] >> *m;
        f.addData(slot, m);
        // BOOST_LOG_TRIVIAL(debug) << " putMat " << slot;
    } else {
        BOOST_LOG_TRIVIAL(warning) << " putMat COULD NOT FIND " << slot;
    }
}

static inline void putInt(Frame& f, cv::FileStorage& file,
                          const std::string& slot)
{
    if (!file[slot].isNone()) {
        int i;
        file[slot] >> i;
        f.addData(slot, i);
        // BOOST_LOG_TRIVIAL(debug) << " putInt " << slot;
    } else {
        BOOST_LOG_TRIVIAL(warning) << " putInt COULD NOT FIND " << slot;
    }
}

static inline void putUInt(Frame& f, cv::FileStorage& file,
                           const std::string& slot, unsigned int def = 0)
{
    if (!file[slot].isNone()) {
        int i;
        file[slot] >> i;
        f.addData(slot, (unsigned int)i);
        // BOOST_LOG_TRIVIAL(debug) << " putUInt " << slot;
    } else {
        BOOST_LOG_TRIVIAL(info)
            << " putUInt COULD NOT FIND " << slot << " using default " << def;
        f.addData(slot, def);
    }
}

bool ImportYaml::loadFromYaml(Frame& f, const std::string& idx)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "%s/%s_%s.yaml", path.c_str(), prefix.c_str(),
             idx.c_str());
    BOOST_LOG_TRIVIAL(debug) << id() << "... to open " << buf;

    // Declare what you need
    cv::FileStorage file(buf, cv::FileStorage::READ);

    if (!file.isOpened()) {
        BOOST_LOG_TRIVIAL(warning) << id() << "failed to open " << buf;
        return false;
    }

    putMat(f, file, "x");
    putMat(f, file, "y");
    putMat(f, file, "z");
    putMat(f, file, "ampl");

    putUInt(f, file, "fc");
    putUInt(f, file, "ts", counter);

    counter++;
    return true;
}
