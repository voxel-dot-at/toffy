//
// request_handler.cpp
// ~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <fstream>
#include <sstream>
#include <string>
#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/bind.hpp>

#include "toffy/web/server/mime_types.hpp"
#include "toffy/web/server/reply.hpp"
#include "toffy/web/server/request.hpp"
#include "toffy/web/server/request_handler.hpp"
#include <toffy/web/server/connection.hpp>
#include <toffy/web/server/handlerinterface.hpp>

namespace http {
namespace server {

request_handler::request_handler(const std::string& doc_root)
    : doc_root_(doc_root), _ch(NULL)
{
}

void request_handler::handle_request(connection_ptr con, const request& req, reply& rep) {
    // Decode url to path.
    std::string request_path;
    if (!url_decode(req.uri, request_path))
    {
	rep = reply::stock_reply(reply::bad_request);
	return;
    }

    // Request path must be absolute and not contain "..".
    if (request_path.empty() || request_path[0] != '/'
	    || request_path.find("..") != std::string::npos)
    {
	rep = reply::stock_reply(reply::bad_request);
	boost::asio::async_write(con->socket(), rep.to_buffers(),
		boost::bind(&connection::handle_write, con,
		  boost::asio::placeholders::error));
	return;
    }

    // check if path points to an existing file.
    // if yes, go to file handler, if no got to toffy/web handler:

    std::string full_path = doc_root_ + request_path;

    if ( request_path.find("/toffy/web") != std::string::npos) {
        handle_toffy_web_request(con, req, rep);
    } else {
	// check for optional arguments
	std::size_t q_pos = full_path.find("?");
	if ( q_pos != std::string::npos) {
	    full_path = full_path.substr(0, q_pos); // strip arguments.
	}

	// check for dir. index:
	if ( full_path[ full_path.size()-1 ] == '/')
	    full_path += "index.html";

	bool success = handle_file_request(req, rep, full_path);

	if (!success) { // fallback
	    handle_toffy_web_request(con, req, rep);
	} else {
	    boost::asio::async_write(con->socket(), rep.to_buffers(),
		    boost::bind(&connection::handle_write, con,
		      boost::asio::placeholders::error));
	}
    }
}

void request_handler::handle_toffy_web_request(connection_ptr &con, const request& req, reply& rep)
{
    _ch->processRequest(con, req, rep);
    
    // Fill out the reply to be sent to the client.
    rep.status = reply::ok;
    
    if (req.uri.find("jpg") != std::string::npos ||
	    req.uri.find("cloud") != std::string::npos ||
	    req.uri.find("actions") != std::string::npos) {
	//std::cout << "Do nothing."<< std::endl;
    } else {
	rep.headers.resize(3);
	rep.headers[0].name = "Content-Length";
	rep.headers[0].value = boost::lexical_cast<std::string>(rep.content.size());
	rep.headers[1].name = "Content-Type";
	rep.headers[1].value = mime_types::extension_to_type("json");
	rep.headers[2].name = "Access-Control-Allow-Origin";
	rep.headers[2].value = "*";
	boost::asio::async_write(con->socket(), rep.to_buffers(),
		boost::bind(&connection::handle_write, con,
		  boost::asio::placeholders::error));
    }
    /*boost::asio::async_write(con.socket(), rep.to_buffers(),
	boost::bind(&connection::handle_write_nostop, shared_from_this(),
	  boost::asio::placeholders::error));*/

}

// quick check if it's an existing directory
// returns false on error
static inline bool isDir(const char* f, bool& isADir)
{
    isADir = true;
    boost::filesystem::path fsPath(f);
    if ( !boost::filesystem::exists(fsPath)) {
	BOOST_LOG_TRIVIAL(debug) << "Given path " << fsPath
				   << " does not exist.";
	isADir = false;
	return true;
    }


    if ( !boost::filesystem::is_directory(fsPath)) {
	/*BOOST_LOG_TRIVIAL(warning) << "Given path " << fsPath
				     << " is not a directory.";*/
	isADir = false;
	return true;
    }

    return false;
}


bool request_handler::handle_file_request(const request& req, reply& rep,
					  const std::string& full_path)
{
    bool isADir=false;
    bool exists = isDir(full_path.c_str(), isADir);

    if (!exists) {
	return false;
    }
    std::string fp = full_path;
    if (isADir) {
	fp += "/index.html";
    }

    // Determine the file extension.
    std::size_t last_slash_pos = fp.find_last_of("/");
    std::size_t last_dot_pos = fp.find_last_of(".");
    std::string extension;

    if (last_dot_pos != std::string::npos && last_dot_pos > last_slash_pos) {
	extension = fp.substr(last_dot_pos + 1);
    }

    // Open the file to send back.
    std::ifstream is(fp.c_str(), std::ios::in | std::ios::binary);
    if (!is) {
	rep = reply::stock_reply(reply::not_found);
	return false;
    }

    // Fill out the reply to be sent to the client.
    rep.status = reply::ok;

    char buf[512];
    while (is.read(buf, sizeof(buf)).gcount() > 0)
	rep.content.append(buf, is.gcount());

    // headers:
    rep.headers.resize(3);
    rep.headers[0].name = "Content-Length";
    rep.headers[0].value = boost::lexical_cast<std::string>(rep.content.size());
    rep.headers[1].name = "Content-Type";
    rep.headers[1].value = mime_types::extension_to_type( extension );
    rep.headers[2].name = "Access-Control-Allow-Origin";
    rep.headers[2].value = "*";

    return true;
}

bool request_handler::url_decode(const std::string& in, std::string& out)
{
    out.clear();
    out.reserve(in.size());
    for (std::size_t i = 0; i < in.size(); ++i)
    {
	if (in[i] == '%')
	{
	    if (i + 3 <= in.size())
	    {
		int value = 0;
		std::istringstream is(in.substr(i + 1, 2));
		if (is >> std::hex >> value)
		{
		    out += static_cast<char>(value);
		    i += 2;
		}
		else
		{
		    return false;
		}
	    }
	    else
	    {
		return false;
	    }
	}
	else if (in[i] == '+')
	{
	    out += ' ';
	}
	else
	{
	    out += in[i];
	}
    }
    return true;
}

} // namespace server
} // namespace http
