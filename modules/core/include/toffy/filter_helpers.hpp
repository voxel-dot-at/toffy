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

#include <cctype>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/log/trivial.hpp>

// for silencing warnings of unused parameters, use UNUSED(param); as in:
#define UNUSED(expr) do { (void)(expr); } while (0)

/* enable nice(r) logging by providing a short-hand version
 */
#define LOG(lvl)    BOOST_LOG_TRIVIAL(lvl) << id() << "::" << __FUNCTION__ << "() : "

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
    const std::string& key, T& val, const T& defaultValue) 
{
    boost::optional<T> opt = pt.get_optional<T>(key);
    if (opt.is_initialized()) {
        val = *opt;
        return true;
    }
    val = defaultValue;
    return false;
}

namespace toffy {

/**
 * Is this configuration string safe to hand to `snprintf` as a *format*?
 *
 * File-name patterns (`options/pattern`, `options/depthPattern`, ...) come
 * straight out of XML and are used as the format argument of
 * `snprintf(buf, size, pattern.c_str(), frameCounter)`. A format that does not
 * match its single `int` argument is undefined behaviour, and `-Wformat` cannot
 * help because the format is not a literal: `%s` reads a pointer out of the
 * stack slot that holds the counter. Validate once, at config time, instead of
 * trusting every config file in the wild to be a C format-string expert.
 *
 * Accepted: no conversion at all, or exactly one signed-decimal conversion
 * (`%d` or `%i`, with optional flags, field width and precision), plus `%%`
 * literals. That covers the documented usage (`data/%05d_d.csv`) and rejects
 * `%s`, `%f`, `%x` and any pattern with more than one conversion.
 *
 * `why` receives the reason when this returns false, so callers can log
 * something a user can act on. Finding N5, work item `P3-6`; the two filters
 * that expand patterns (`CSVSource`, `ExportCSV`) share this validator.
 */
inline bool validSequencePattern(const std::string& pattern, std::string& why) {
    int conversions = 0;
    for (std::size_t i = 0; i < pattern.size(); ++i) {
        if (pattern[i] != '%') continue;
        ++i;                                                    // skip the '%'
        if (i >= pattern.size()) {
            why = "pattern ends in a bare '%'";
            return false;
        }
        if (pattern[i] == '%') continue;                        // "%%" is a literal percent
        while (i < pattern.size() && std::strchr("-+ 0#", pattern[i])) ++i;
        while (i < pattern.size() &&
               std::isdigit(static_cast<unsigned char>(pattern[i]))) ++i;
        if (i < pattern.size() && pattern[i] == '.') {
            ++i;
            while (i < pattern.size() &&
                   std::isdigit(static_cast<unsigned char>(pattern[i]))) ++i;
        }
        if (i >= pattern.size()) {
            why = "pattern ends inside a conversion";
            return false;
        }
        const char conv = pattern[i];
        if (conv != 'd' && conv != 'i') {
            why = std::string("'%") + conv +
                  "' is not a signed-decimal conversion; the only argument supplied "
                  "is the frame counter (an int)";
            return false;
        }
        if (++conversions > 1) {
            why = "more than one conversion; only the frame counter is supplied";
            return false;
        }
    }
    return true;
}

/**
 * Expand a file-name pattern and report truncation instead of acting on it.
 *
 * Returns false when the expansion did not fit in `size`; `buf` then holds the
 * truncated result, which used to be opened as if it were the intended file.
 * Every pattern expansion in the library goes through here, so there is one
 * place where the format argument is a variable rather than a literal.
 */
inline bool formatPath(char* buf, std::size_t size, const std::string& fmt,
                       int sequence) {
    const int n = snprintf(buf, size, fmt.c_str(), sequence);
    return n >= 0 && static_cast<std::size_t>(n) < size;
}

}  // namespace toffy

static inline bool pt_optional_get_ipaddr(const boost::property_tree::ptree pt,
    const std::string& key, struct in_addr& inaddr, std::string defaultAddress) 
{
    boost::optional<std::string> opt = pt.get_optional<std::string>(key);
    std::string addr = defaultAddress;
    if (! opt.is_initialized() && defaultAddress.size() == 0) {
        return false;
    }
    if (opt.is_initialized()) {
        addr = *opt;
    }
    int success = inet_aton( addr.c_str(), &inaddr);
    if (!success) {
        BOOST_LOG_TRIVIAL(warning) << "pt_optional_get_ipaddr() could not parse entry " << key << " : " << addr;
    } else {
        BOOST_LOG_TRIVIAL(info) << "pt_optional_get_ipaddr set " << key << " " << addr;
    }
    return success == 1;
}
