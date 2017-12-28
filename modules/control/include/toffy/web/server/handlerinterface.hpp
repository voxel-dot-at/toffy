#ifndef HTTP_HANDLER_HPP
#define HTTP_HANDLER_HPP

#include <string>

#include "toffy/web/server/request.hpp"
#include "toffy/web/server/reply.hpp"
#include "toffy/web/server/connection.hpp"

namespace http {
namespace server {

class connection;

/**
 * @brief Interface for http request handlers.
 *
 * The class inheriting from this class can be add to the request_handler and
 * get the connection and request objects to provides the desidered reply.
 *
 */
class Handler
{
public:
    /**
    * @brief Process the request sent in the connection
    * @param con
    * @param req
    * @param rep
    * @return Positice on sucess, negative or 0 in failed
    *
    * The classes that implement this method should make use of the connection
    * and are responsable for sending the repy and finishing the connection if
    * requiered.
    *
    * @todo does Make sense to send the reply with the new con responsible aproach?
    */
   virtual int processRequest(connection_ptr con, const request& req, reply &rep)=0;
};
} // namespace server
} // namespace http

#endif // HTTP_HANDLER_HPP
