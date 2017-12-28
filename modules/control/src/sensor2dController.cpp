#include <toffy/web/sensor2dController.hpp>
#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>
//#include <toffy/sensor2d.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace toffy {
namespace control {
using namespace std;

bool Sensor2dController::doAction(Action &action, std::string &log) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;

    std::stringstream ss;
    boost::property_tree::ptree jOutput;

    /*toffy::capturers::Sensor2 *sensor2d = dynamic_cast<toffy::capturers::Sensor2*>(filter());

    string al1, al2; //Action levels defined /toffy/al1/al2
    if (action._items.size() > 2)
	al1 = action._items[2];

    BOOST_LOG_TRIVIAL(debug) << "action._items[2], name: " << al1;
    string input = action.req->content;
    if (al1.empty()) {
	if (!input.empty()) {
	    BOOST_LOG_TRIVIAL(debug) << "Content: " << input;
	}
	// read config, create json
	boost::property_tree::ptree vals;
	boost::property_tree::ptree inputs;
	boost::property_tree::ptree outputs;

	jOutput = sensor2d->getConfig();
	cout << "hola" << endl;
	jOutput.add("connected",sensor2d->getSensor()->isConnected());
	jOutput.add("playBack",sensor2d->playback());
	jOutput.add("save",sensor2d->save());

    } else if (al1 == "connect") {
	if (sensor2d->getSensor()->start() < 0) {
	    log = "Could not connect.";
	    BOOST_LOG_TRIVIAL(debug) << log;
	    return false;
	} else {
	    BOOST_LOG_TRIVIAL(debug) << "Connected.";
	    sensor2d->save(sensor2d->save());
	    // Dealing with saving:

	}
    } else if (al1 == "disconnect") {

	sensor2d->getSensor()->stop();

    } else if (al1 == "playback") {
	BOOST_LOG_TRIVIAL(debug) << "playback";
	if (action._items.size() >= 4) {
	    al2 = action._items[3];

	    std::stringstream ssInput(input);
	    boost::property_tree::ptree vals;

	    BOOST_LOG_TRIVIAL(debug) << "Content: " << input;
	    boost::property_tree::json_parser::read_json(ssInput,vals);

	    boost::property_tree::ptree::const_assoc_iterator it = vals.find("loadPath");
	    if (it != vals.not_found()) {
		sensor2d->loadPath(it->second.data());
	    }
	    it = vals.find("fileBegin");
	    if(it != vals.not_found()) {
		sensor2d->beginFile(boost::lexical_cast<int>(it->second.data()));
	    }
	    it = vals.find("fileEnd");
	    if(it != vals.not_found()) {
		sensor2d->endFile(boost::lexical_cast<int>(it->second.data()));
	    }

	    if(al2 == "start") {
		sensor2d->save(false);
		sensor2d->playback(true);
		//TODO fileBegin should define this
		sensor2d->cnt(1);
	    } else if(al2 == "stop") {
		sensor2d->playback(false);
	    } else {
		jOutput.add("status", "failed");
		jOutput.add("reason", "Unknown action.");
		ss.clear();
		boost::property_tree::json_parser::write_json(ss,jOutput);
		log = ss.str();
		return false;
	    }
	}

	jOutput.add("loadPath",sensor2d->loadPath());
	jOutput.add("fileBegin",sensor2d->beginFile());
	jOutput.add("fileEnd",sensor2d->endFile());
	jOutput.add("playBack",sensor2d->playback());
	jOutput.add("type", sensor2d->type());

    } else if (al1 == "control") {

	if (sensor2d->getSensor()->isConnected()) {

	    if (!action.req->content.empty()) {
		std::stringstream ssInput(action.req->content);
		boost::property_tree::ptree vals;

		BOOST_LOG_TRIVIAL(debug) << "Content: " << input;
		boost::property_tree::json_parser::read_json(ssInput,vals);

//		boost::property_tree::ptree::const_assoc_iterator it = vals.find("it");
//		if (it != vals.not_found()) {
//		    sensor2d->getSensor()->getCamera().setIntegrationTime(boost::lexical_cast<int>(it->second.data()));
//		}
//		it = vals.find("fr");
//		if(it != vals.not_found()) {
//		    sensor2d->getSensor()->getCamera().setFrameRate(boost::lexical_cast<float>(it->second.data()));
//		}
//		it = vals.find("fr");
//		if(it != vals.not_found()) {
//		    sensor2d->getSensor()->getCamera().setModulationFrequency(boost::lexical_cast<int>(it->second.data()));
//		}
	    }
	    //jOutput.add("it",sensor2d->getSensor()->getCamera().getIntegrationTime());
	    //jOutput.add("mf",sensor2d->getSensor()->getCamera().getModulationFrequency());
	    //jOutput.add("fr",sensor2d->getSensor()->getCamera().getFrameRate());

	}
    } else if (al1 == "save") {
	if (action._items.size() >= 4) {
	    al2 = action._items[3];
	    if (sensor2d->playback())
		sensor2d->save(false);
	    else {
		std::stringstream ssInput(input);
		boost::property_tree::ptree vals;

		BOOST_LOG_TRIVIAL(debug) << "Content: " << input;
		boost::property_tree::json_parser::read_json(ssInput,vals);

		boost::property_tree::ptree::const_assoc_iterator it = vals.find("savePath");
		if (it != vals.not_found()) {
		    sensor2d->savePath(it->second.data());
		}
		it = vals.find("saveFolder");
		if(it != vals.not_found()) {
		    sensor2d->saveFolder(it->second.data());
		}
		it = vals.find("timeStamped");
		if(it != vals.not_found()) {
		    sensor2d->timeStamped(true);
		} else
		    sensor2d->timeStamped(false);
		if(al2 == "start") {
		    if (!sensor2d->playback()) {
			sensor2d->save(true);
		    }
		} else if(al2 == "stop") {
		    sensor2d->save(false);
		} else {
		    jOutput.add("status", "failed");
		    jOutput.add("reason", "Unknown action.");
		    ss.clear();
		    boost::property_tree::json_parser::write_json(ss,jOutput);
		    log = ss.str();
		    return false;
		}
	    }
	}
	jOutput.add("save",sensor2d->save());
	jOutput.add("savePath",sensor2d->getSavePath());
	jOutput.add("saveFolder",sensor2d->saveFolder());
	jOutput.add("timeStamped",sensor2d->timeStamped());

    } else {
	return FilterController::doAction(action,log);
    }

    */
    jOutput.add("status","ok");
    boost::property_tree::json_parser::write_json(ss,jOutput);
    log = ss.str();
    return true;
}
} // namespace control
} // namespace toffy
