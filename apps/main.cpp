/**
 *
 * @file main.cpp
 * @example main.cpp
 * @brief Minimal example application of L3vl
 *
 *
 */
#include <iostream>

#include <toffy/player.hpp>

#if OCV_VERSION_MAJOR >= 3
#  include <opencv2/highgui.hpp>
#else
#  include <opencv2/highgui/highgui.hpp>
#endif

#ifdef MSVC
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

BOOL WINAPI ConsoleHandler(DWORD);
#else
#include <signal.h>
#endif

#include <boost/program_options.hpp>
static bool keepRunning = false;
using namespace boost;
namespace po = boost::program_options;
using namespace std;

#ifdef MSVC
BOOL WINAPI ConsoleHandler(DWORD dwType)
{
    switch(dwType) {
    case CTRL_C_EVENT:
    case CTRL_BREAK_EVENT:
        if (keepRunning)
            keepRunning = false;
        else
            exit(1);
        break;
    default:
        printf("Other event\n");
    }
    return TRUE;
}
#else
void my_handler(int s){
    //printf("Caught signal %d\n",s);
    cout << "Ctrl-c detected. Stopping application...." << endl;
    if (keepRunning)
	keepRunning = false;
    else
	exit(1);
}
#endif

int main(int argc, char* argv[])
{
    //// Reading parameters ////
    po::variables_map vm;
    try {

	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("host,h", po::value<std::string>()->default_value("localhost"), "Server host address")
		("port,p", po::value<std::string>()->default_value("9999"), "Port")
		("html,d", po::value<std::string>()->default_value("/opt/toffy/html/"), "Folder path containg the web interface")
		("config,c", po::value<std::string>()->default_value("config.xml"), "Path to the config file")
		("sleepDelay,s", po::value<int>()->default_value(30), "ms to wait between frames (0 for keypress)")
		("output2File,f", po::value<bool>()->default_value(false), "Set program output to a file, silence console")
		//("extensions,e", po::value< std::vector<std::string> >(), "Extensions")
		;


	po::store(po::command_line_parser(argc, argv).
		  options(desc).run(), vm);
	po::notify(vm);

	if (vm.count("help")) {
	    cout << "Usage: options_description [options]\n";
	    cout << desc;
	    return 0;
	}

	cout << "Host: " << vm["host"].as<std::string>() << endl;
	cout << "Port: " << vm["port"].as<std::string>() << endl;
	cout << "Html path: " << vm["html"].as<std::string>() << endl;
	cout << "Config file: " << vm["config"].as<std::string>() << endl;
	cout << "Sleep delay: " << vm["sleepDelay"].as<int>() << endl;
	cout << "output2File: " << vm["output2File"].as<bool>() << endl;
	/*if (vm.count("configfile"))
	{
	    cout << "Configfile: " << vm["configfile"].as<std::string>() << endl;
	} else {
	    cout << "Missing configFile!" << endl;
	    cout << "Usage: options_description [options]\n";
	    cout << desc;
	    return 0;
	}*/

    } catch(std::exception& e) {
	cout << e.what() << "\n";
	return 1;
    } catch(...) {
	cerr << "Exception of unknown type!\n";
    }
    //// End reading parameters ////
    try
    {
	// Initialise the server.
	cout << "INIT PLAYER" << endl;
	toffy::Player p(boost::log::trivial::info,vm["output2File"].as<bool>());

	p.loadConfig(vm["config"].as<std::string>());
	std::cout << "Player done" << std::endl;

#ifdef MSVC
    SetConsoleCtrlHandler((PHANDLER_ROUTINE)ConsoleHandler,TRUE);
#else

	const int delay = vm["sleepDelay"].as<int>();

	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = my_handler;
	//sigemptyset(&sigIntHandler.sa_mask);
	//sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);
#endif
	keepRunning = true;
	int c=' ';
	do {
	    p.runOnce();

	    c = cv::waitKey(delay);
        //boost::this_thread::sleep( boost::posix_time::milliseconds(30) );
	    //cout << "KEY " << c << "\t" << (char)c << endl;
	    keepRunning = c != 'q' ;
	} while (keepRunning);
	std::cout << "Stopped..." << std::endl;

    }
    catch (std::exception& e)
    {
	std::cerr << "exception: " << e.what() << "\n";
    }

    return 0;
}
