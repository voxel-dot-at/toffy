/**
 * @file request_handler.hpp
 *
 * from:
 * Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
 *
 * Distributed under the Boost Software License, Version 1.0. (See accompanying
 * file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
 *
 * Boost Asio http server request handler adpated from boost examples.
 *
 */
#ifndef HTTP_REQUEST_HANDLER_HPP
#define HTTP_REQUEST_HANDLER_HPP

#include <string>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

namespace http {
namespace server {

struct reply;
struct request;
class connection;
typedef boost::shared_ptr<connection> connection_ptr;
class Handler;

/// The common handler for all incoming requests.
///
/// Handles http requestes for files of json encoded toffy/web actions
///
class request_handler: private boost::noncopyable
{
public:
    /// Construct with a directory containing files to be served.
    explicit request_handler(const std::string& doc_root);

    /// Handle a request and produce a reply.
    void handle_request(const request& req, reply& rep);

    /// Handle a request and produce a reply.
    void handle_request(connection_ptr con, const request& req, reply& rep);

    /**
     * @brief Setter for the request Handler
     * @param ch
     *
     * @todo so far there is just one.
     * Move to a vector for more/different handlers?
     */
    void setHandler(Handler *ch) {_ch = ch;}

private:
    /// The directory containing the files to be served.
    std::string doc_root_;
    http::server::Handler *_ch;
    /// Perform URL-decoding on a string. Returns false if the encoding was
    /// invalid.
    static bool url_decode(const std::string& in, std::string& out);


    /// Handle a request to toffy/web and produce a reply.
    void handle_toffy_web_request(connection_ptr &con, const request& req, reply& rep);

    /// Handle a request to toffy/web and produce a reply.
    /// @return true if the request could be serviced.
    bool handle_file_request(const request& req, reply& rep, const std::string& full_path);
};

} // namespace server
} // namespace http

#endif // HTTP_REQUEST_HANDLER_HPP
