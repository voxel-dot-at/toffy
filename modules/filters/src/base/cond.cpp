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
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>



#include "toffy/base/cond.hpp"

using namespace toffy;
using namespace toffy::filters;
using namespace cv;
using namespace std;

static std::string theName("cond");
static int counter = 0;

Cond::Cond() : FilterBank(theName, counter++), enabled(false), opt_file("cond.xml") {}

bool Cond::filter(const Frame& in, Frame& out)
{
    LOGD << __FUNCTION__ << " " << id();

    if (enabled) {
        return FilterBank::filter(in, out);
    }

    return true;
}

boost::property_tree::ptree Cond::getConfig() const
{
    boost::property_tree::ptree pt;

    pt = Filter::getConfig();

    pt.put("options.file", opt_file);
    pt.put("options.enabled", enabled);

    return pt;
}

void Cond::updateConfig(const boost::property_tree::ptree& pt)
{
    LOGD << __FUNCTION__ << " " << id();

    using namespace boost::property_tree;

    Filter::updateConfig(pt);

    enabled = pt.get<bool>("options.enabled", enabled);

    LOGD << __FUNCTION__ << " " << id() << " enabled? " << enabled;
}

int Cond::loadConfig(const boost::property_tree::ptree& pt, const std::string& confFile)
{

    LOGD << "Cond::loadConfig " << id() << " " << confFile << " " << pt.begin()->first;
    if (pt.begin()->first != type()) {
        // on sub-filterbanks that are loaded by Cond, the overloaded loadConfig is called ... pass the call up to where it belongs: 
        LOGD << "Cond::loadConfig " << id() << " .. not for us, passing up.";
        return FilterBank::loadConfig(pt, confFile);
    }

    const boost::property_tree::ptree& self = pt.get_child( "cond" );

    enabled = self.get<bool>("options.enabled", enabled);
    LOGD << "Cond::loadConfig " << id() << " enabled? " << enabled;
    boost::optional<const boost::property_tree::ptree& > pfilters = self.get_child_optional( "filterBank" );

    if( pfilters ) {
        LOGD << "Cond::loadConfig " << id() << " has filterBank :)";

        return FilterBank::loadConfig(confFile, pfilters->begin(), pfilters->end());
    }
    boost::optional<const boost::property_tree::ptree& > pfile = self.get_child_optional( "file" );
    if( pfile ) {
        LOGD << "Cond::loadConfig " << id() << " has file :) " << pfile->data();

        return FilterBank::loadFileConfig(pfile->data());
    }

    LOGW << "Cond::loadConfig " << id() << " no dependent filters defined! Use filterBank or file!" ;

    return 0;
}
