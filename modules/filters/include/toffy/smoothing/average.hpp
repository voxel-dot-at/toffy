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
#pragma once

#include <toffy/filter.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <deque>

#ifdef MSVC
#define DLLExport __declspec(dllexport)
#else
#define DLLExport /**/
#endif

/**
 * @brief
 *
 */
namespace toffy {
namespace filters {
namespace smoothing {
/** perform averaging of the (depth) channel over multiple frames.
     */
class DLLExport Average : public Filter
{
    std::string _in_img, _out_img;
    static std::size_t _filter_counter;
    std::deque<matPtr> _queue;
    size_t _size;
    cv::Mat _dst, _cnt;

   public:
    Average();

    virtual ~Average() {}

    static const std::string id_name;

    //virtual int loadConfig(const boost::property_tree::ptree& pt);

    virtual bool filter(const Frame& in, Frame& out);

    virtual boost::property_tree::ptree getConfig() const;

    void updateConfig(const boost::property_tree::ptree& pt);

    size_t size() const { return _queue.size(); }
};
}  // namespace smoothing
}  // namespace filters
}  // namespace toffy
