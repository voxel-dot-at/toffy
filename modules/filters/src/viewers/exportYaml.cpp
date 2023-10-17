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
#include <pcl/range_image/range_image_planar.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core.hpp>

#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/any.hpp>

#include "toffy/filter_helpers.hpp"
#include "toffy/viewers/exportYaml.hpp"

using namespace toffy;
using namespace cv;

void ExportYaml::updateConfig(const boost::property_tree::ptree& pt)
{
    LOG(debug) << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    path = pt.get("options.path", path);
    prefix = pt.get("options.prefix", path);

    useCounter = pt.get<bool>("options.useCounter", useCounter);
    useFc = pt.get<bool>("options.useFc", useFc);

    _in_cloud = pt.get<std::string>("inputs.cloud", _in_cloud);
}

boost::property_tree::ptree ExportYaml::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.path", path);
    pt.put("options.prefix", prefix);

    pt.put("options.useCounter", useCounter);
    pt.put("options.useFc", useFc);

    pt.put("inputs.cloud", _in_cloud);

    return pt;
}

bool ExportYaml::filter(const Frame& in, Frame& out)
{
    UNUSED(out);
    
    LOG(debug) << " exporting " << _in_cloud;
    char buf[80] = "";
    unsigned int fc = in.optUInt("fc", 1000);

    if (useCounter) {
        snprintf(buf,sizeof(buf),"%04d", counter);
    }
    if (useFc) { // TODO fc()
        snprintf(buf,sizeof(buf),"%04d", fc);
    }

    dumpToYaml(in, std::string(buf));

    counter++;
    return true;
}

void ExportYaml::dumpToYaml(const Frame& frame, const std::string& id)
{
    // also dump amplitudes..
    char buf[128];
    snprintf(buf, sizeof(buf), "%s/%s_%s.yaml", path.c_str(),
             prefix.c_str(), id.c_str());

    // Declare what you need
    cv::FileStorage file(buf, cv::FileStorage::WRITE);
    // Write to file!
    if (frame.hasKey("x")) {
        file << "x" << * frame.getMatPtr("x");
    }
    if (frame.hasKey("y")) {
        file << "y" << * frame.getMatPtr("y");
    }
    if (frame.hasKey("z")) {
        file << "z" << * frame.getMatPtr("z");
    }
    if (frame.hasKey("depth")) {
        file << "depth" << * frame.getMatPtr("depth");
    }
    if (frame.hasKey("ampl")) {
        file << "ampl" << * frame.getMatPtr("ampl");
    }
    if (frame.hasKey("confidence")) {
        file << "confidence" << * frame.getMatPtr("confidence");
    }

    // frame info:
    unsigned int fc = frame.optUInt("fc", 1000);
    file << "fc" << (int)fc;

    file << "counter" << counter;

    // if (in.hasKey("color0")) {
    //     char buf[128];
    //     snprintf(buf, sizeof(buf), "%s/mats_%s.rgb.jpg",
    //              systemGeometry.path.c_str(), id.c_str());
    //     vector<int> p(2);
    //     p[0] = IMWRITE_JPEG_QUALITY;
    //     p[1] = 95;  // compression factor

    //     imwrite(buf, *in.getMatPtr("color0"), p);
    // }
    // BOOST_LOG_TRIVIAL(info) << "FULL FIN ORGA DUMPED " << buf;
}
