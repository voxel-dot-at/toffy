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
#include <string>
#include <vector>

#include "toffy/filter.hpp"

namespace toffy
{
    /**
     * @brief class Mux multiplexes multiple input frames and creates one
     * output frame.
     * @ingroup Core
     *
     */
    class Mux: public Filter
    {
    public:
	/**
	 * @brief Mux
	 * @param name
	 *
	 */
	Mux(const std::string& name);

	virtual ~Mux();

	/**
	 * @brief The Filter::filter() is overwrite to treat a single input
	 * frame as a vector of frames with a single element.
	 * @param in
	 * @param out
	 * @return result of Mux::filter(const std::vector<Frame*>& in,
	 *				Frame& out)
	 */
	virtual bool filter(const Frame& in, Frame& out) override;

	//virtual int loadConfig(const boost::property_tree::ptree& pt)=0;
	//virtual void updateConfig(const boost::property_tree::ptree &pt)=0;

	/**
	 * @brief Extension of the Filter::filter() method to work with a group
	 * of input frames and merge all in a single out Frame.
	 * @param in std::vector<Frame*>&
	 * @param out
	 * @return True on success, False if failed.
	 */
	virtual bool filter(const std::vector<Frame*>& in, Frame& out) = 0;

    protected:
    };
}
