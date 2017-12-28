#include <boost/log/trivial.hpp>

#include <toffy/player.hpp>

#include <toffy/web/webcontroller.hpp>

using namespace toffy;
using namespace toffy::control;
using namespace std;

/*Controller * Controller::_controller;


Controller * Controller::getInstance() {
    if(_controller == NULL) {
	_controller = new Controller();
    }
    return _controller;
}*/

WebController::~WebController() {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    if (_thread.joinable()) {
      _server->stop();
      //_thread.join();
    }
}

void WebController::startServer(std::string address, std::string port,
			 std::string doc_root)
{
    _server = new http::server::server (
		address, port, doc_root);
    ControllerFactory::getInstance()->setCtrl(_p->getController());
    _controller.setCtrl(_p->getController());
    _server->getRequestHandler().setHandler(&_controller);
    _thread = boost::thread(boost::bind(&WebController::runServer,this));
}

void WebController::runServer() {
    try
      {
	_server->run();
	//BOOST_LOG_TRIVIAL(warning) << "Finished _server->run()";
      }
      catch (std::exception& e)
      {
	std::cerr << "exception: " << e.what() << "\n";
      }

}

void WebController::stopServer() {
    _server->stop();
    _thread.join();

}
