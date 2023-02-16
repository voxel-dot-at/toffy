/*
   Copyright 2023 Simon Vogl <simon@voxel.at>
                 
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

/** misc helper functions for implementing filters. 
 * Include this in your cpp file, not in the header.
 */

 
#include <arpa/inet.h> // inet_aton

#include <boost/property_tree/xml_parser.hpp>
#include <boost/log/trivial.hpp>

/** optionally get a value from the property tree if it exists.
 * @return true if key exists and the value has been set, false otherwise
 */
template<typename T> bool pt_optional_get(const boost::property_tree::ptree pt,
    const std::string& key, T& val) 
{
    boost::optional<T> opt = pt.get_optional<T>(key);
    if (opt.is_initialized()) {
        val = *opt;
        return true;
    } else {
        // BOOST_LOG_TRIVIAL(debug) << "pt_optional_get - key not set: " << key;
    }
    return false;
}

/** optionally get a value from the property tree if it exists.
 * @return true if key exists and the value has been set, false otherwise
 */
template<typename T> bool pt_optional_get_default(const boost::property_tree::ptree pt,
    const std::string& key, T& val, T defaultValue) 
{
    boost::optional<T> opt = pt.get_optional<T>(key);
    if (opt.is_initialized()) {
        val = *opt;
        return true;
    }
    val = defaultValue;
    return false;
}

static inline bool pt_optional_get_ipaddr(const boost::property_tree::ptree pt,
    const std::string& key, struct in_addr& inaddr, std::string defaultAddress) 
{
    boost::optional<std::string> opt = pt.get_optional<std::string>(key);
    std::string addr = defaultAddress;
    if (! opt.is_initialized() && defaultAddress.size() == 0) {
        return false;
    }
    if (opt.is_initialized()) {
        std::string addr = *opt;
    }
    int success = inet_aton( addr.c_str(), &inaddr);
    if (!success) {
        BOOST_LOG_TRIVIAL(warning) << "pt_optional_get_ipaddr() could not parse entry " << key << " : " << addr;
    }
    return success == 1;
}
