/*
   Copyright 2012-2021 Simon Vogl <svogl@voxel.at> VoXel Interaction Design - www.voxel.at

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

#include <boost/any.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/optional.hpp>

#include <toffy/web/server/request.hpp>

typedef std::string::const_iterator iterator_type;

namespace toffy {
namespace control {

/**
 * @brief Parser of a request into readable actions and configs
 * @ingroup Control
 *
 * Gets a http::server::request and parse it using boost::spirit and
 * boost::fusion. The outputs are vector with the Rest action ordered and
 * key-value ordered lists for request parameters and content values.
 *
 */
class Action {

public:
    const http::server::request * req; ///< request parsed in Action
    std::vector<std::string> _items; ///< list of url names
    boost::container::flat_map< std::string, boost::optional<std::string> > _parameters; ///< Key-value list of uri parameters

    Action(): req(NULL) {}

    /**
     * @brief Constructor with request
     * @param request http request object.
     *
     * Decodes the url automatically.
     */
    Action(const http::server::request &request);

    virtual ~Action() {}

    /**
     * @brief Parse the url in Rest path/action and the arguments
     * @param in url string
     * @return True on success, false if failed
     */
    bool url_decode(std::string in);

    /**
     * @brief Parses the url path part of the uri, separated from parameters
     * @param begin form list of url slash separated names
     * @param end form list of url slash separated names
     * @return True on success, false if failed
     */
    bool path_parse(iterator_type begin, const iterator_type &end);

    /**
     * @brief Parses the uri parameters
     * @param begin form list of uri parameters coma separated elements.
     * @param end form list of uri parameters coma separated elements.
     * @return True on success, false if failed
     */
    bool params_parse( iterator_type begin, const iterator_type &end);

};

} // namespace controller
} // namespace toffy
