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

#include <vector>

#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/container/flat_map.hpp>

#include <opencv2/core.hpp>

#include "toffy/frame.hpp"

namespace toffy
{
    const std::string btaMf = "mf"; ///< constant for modulation frequency
    const std::string btaIt = "it"; ///< constant for integration time
    const std::string btaFc = "fc"; ///< constant for frame counter

    const std::string btaDepth = "depth"; ///< constant for depth mat
    const std::string btaAmpl = "ampl"; ///< constant for amplitudes mat

    class TOFFY_EXPORT BtaFrame : public Frame {

    public:
	BtaFrame() {}
	BtaFrame(const Frame& f): Frame(f) {}

	virtual ~BtaFrame() {}

	matPtr getDepth(const std::string& key = btaDepth) {
	    return getMatPtr(key);
	}
    };
}
