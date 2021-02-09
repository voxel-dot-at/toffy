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

#include <toffy/player.hpp>
#include <toffy/toffy_export.h>
#include <toffy/toffy_config.h>

#include <toffy/web/server/server.hpp>
#include <toffy/web/requestController.hpp>


namespace toffy {

namespace control {

/**
 * @brief This class is responsible for the boost::asio server and for the toffy
 * JSON api.
 * @ingroup Control
 *
 * It starts and stop the server and instantiate the JSON controller. It also
 * bring the toffy controller to the web controllers.
 *
 * @todo move the get web files logic to a separated controller. This way the
 * http::server::request_handler is cleaner. A list of handlers should be added
 * there so that we can add other request parsers.
 */
class TOFFY_EXPORT WebController {
public:

    /**
     * @brief Constructor that forces the WebController link to a player
     * instance
     * @param p Player instance
     */
    WebController(Player *p): _p(p) {}

    virtual ~WebController();

    /**
     * @brief Allows to run the http control server
     * @param addresse IP address
     * @param port Port
     * @param doc_root Location of the html files for control module
     *
     * Only available in control if on
     */
    void startServer(std::string addresse, std::string port,
		     std::string doc_root);

    /**
     * @brief Just stops the server thread and all connections if running.
     */
    void stopServer();


private:
    //static Controller *_controller;
    http::server::server *_server; ///< Instance of the http server
    boost::thread _thread; ///< Thread running the http server
    Player *_p; ///< Pointer to the player
    toffy::control::Controller _controller; ///< The instance to the json controller

    void runServer();
};

} // namespace controller
} // namespace toffy
