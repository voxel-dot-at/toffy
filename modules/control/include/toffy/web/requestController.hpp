#pragma once
#include <string>

#include <toffy/controller.hpp>
#include <toffy/filterbank.hpp>

#include <toffy/web/controllerFactory.hpp>
#include <toffy/web/requestAction.hpp>

#include <toffy/web/server/handlerinterface.hpp>
#include <toffy/web/server/reply.hpp>
#include <toffy/web/server/request.hpp>
#include <toffy/web/server/request_handler.hpp>


/** @defgroup Control Control http
 *
 * Http Control module
 *
 */

namespace toffy {
namespace control {

class ControllerFactory;
/**
 * @brief The controller of the html interface.
 * @ingroup Control
 *
 * This class brings together the http Boost::asio server and the
 * toffy::Controller. It implements the interface that allows to get access to the
 * server connections and requests, and it responsible for giving a reply and
 * handling the connection status.
 *
 * On the other had it should convert the http requests in toffy actions and
 * execute the appropriate commands in toffy through the toffy::Controller.
 *
 */
class Controller: public http::server::Handler {


public:
    Controller(): _c(NULL) {}

    virtual ~Controller();

    /**
     * @brief Set the instance of the running toffy controller
     * @param c Pointer to the toffy
     *
     */
    void setCtrl(toffy::Controller *c) {_c = c;}

    /**
     * @brief Implementation of the interface for handling http requesters.
     * @param con
     * @param req
     * @param rep
     * @return
     *
     * This method will be called after the server gets any valid request. Its
     * responsible for control the connection live cycle and to provide and
     * appropriate reply through that connection.
     */
    virtual int processRequest(http::server::connection_ptr con,
			       const http::server::request& req,
			       http::server::reply &rep);

    /**
     * @brief Provides access to the Frame container.
     * @return Pointer to frame
     *
     */
    toffy::Frame * getFrame() {return &_c->f;}
    toffy::Controller * getController() const {return _c;}


private:

    toffy::Controller *_c; ///< The toffy::Controller
    boost::thread _thread;

    /**
     * @brief Get filters as a ptree in a single list, without hierarchy
     * @param log
     * @param bank
     * @param jsonList Output with the filters list
     *
     */
    void collectFilters(std::string &log, toffy::FilterBank* bank, boost::property_tree::ptree &jsonList);

    /**
     * @brief Get filters as a ptree with hierarchy
     * @param log
     * @param bank
     * @param jsonList Output with the filters tree
     *
     */
    void collectFilterTree(std::string &log, toffy::FilterBank* bank, boost::property_tree::ptree &jsonList);

    /**
     * @brief Helper to load a config file from the request content and create
     * the appropriate reply.
     * @param config File path from the request
     * @param log For the reply
     * @return True if loading file successed, false if failed.
     *
     */
    bool loadConfig(std::string config, std::string &log);

    /**
     * @brief Thread method to handle the mpjeg streaming in thread
     * @param con The connection that should be keeped alive
     * @param req Request
     *
     * This method is launched in a thread to send out images in a simple mpjeg
     * streamer. It will stop when the web browser stops requiring more data
     * (pipe broken).
     *
     */
    void loopImage(http::server::connection_ptr con,
		   const http::server::request &req, http::server::reply &rep,
		   std::string name);

    void loopCloud(http::server::connection_ptr con,
		   const http::server::request &req, http::server::reply &rep);
    void loopAction(http::server::connection_ptr con,
		   const http::server::request &req, http::server::reply &rep);
};

} // namespace controller
} // namespace toffy
