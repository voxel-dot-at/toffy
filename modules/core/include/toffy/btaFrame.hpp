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

#include <boost/container/flat_map.hpp>

#include <opencv2/core.hpp>

#include "toffy/frame.hpp"

namespace toffy
{
    // P2-9: these used to be *defined* right here -- `const std::string btaMf = "mf";`.
    // A namespace-scope `const` object has internal linkage, so every translation unit
    // that included this header got its own std::string, its own dynamic initialiser at
    // startup, and its own address: `&toffy::btaDepth` in one .cpp is not
    // `&toffy::btaDepth` in another. Comparisons still worked, because std::string
    // compares by value -- but this header is installed, so any consumer that took one
    // of these by reference and compared addresses was comparing different objects.
    //
    // Declared extern here, defined once in src/btaFrame.cpp. C++17 inline variables
    // would also do it; `extern` + one .cpp is used because it works on the gcc 7.5
    // toolchain on the deployment target and keeps the strings out of every consumer's
    // object file.
    //
    // Placement: these are camera vocabulary and arguably belong in modules/bta, but
    // the three filters that include this header (blobs, blobsDetector, simpleBlobs)
    // are built unconditionally, so moving the header would make them depend on bta
    // and break -DWITHOUT_BTA=ON. Left in core; recorded as a finding.
    extern const std::string btaMf;    ///< constant for modulation frequency
    extern const std::string btaIt;    ///< constant for integration time
    extern const std::string btaFc;    ///< constant for frame counter

    extern const std::string btaDepth; ///< constant for depth mat
    extern const std::string btaAmpl;  ///< constant for amplitudes mat

    class BtaFrame : public Frame {

    public:
	BtaFrame() {}
	BtaFrame(const Frame& f): Frame(f) {}

	virtual ~BtaFrame() {}

	matPtr getDepth(const std::string& key = btaDepth) {
	    return getMatPtr(key);
	}
    };
}
