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
#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>

#include "toffy/mux.hpp"


using namespace toffy;



Mux::Mux(const std::string& name) : Filter(name) {
}

Mux::~Mux() {
}

bool Mux::filter(const Frame& in, Frame& out)
{
	std::vector<Frame*> l;
	l.push_back((Frame*)&in);
	return filter(l,out);
}
