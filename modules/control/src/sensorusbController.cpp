#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <toffy/filter_helpers.hpp>
#include <toffy/web/sensorusbController.hpp>
//#include <toffy/sensorusb.hpp>

namespace toffy {
namespace control {
using namespace std;

bool SensorUSBController::doAction(Action &action, std::string &log) {
    UNUSED(action);
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    std::stringstream ss;
    boost::property_tree::ptree jOutput;

    /*toffy::capturers::SensorU *SensorUsb = dynamic_cast<toffy::capturers::SensorU*>(filter());

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

	jOutput = SensorUsb->getConfig();
	cout << "hola" << endl;
	jOutput.add("connected",SensorUsb->getSensor()->isConnected());
	jOutput.add("playBack",SensorUsb->playback());
	jOutput.add("save",SensorUsb->save());

    } else if (al1 == "connect") {
	if (SensorUsb->getSensor()->start() < 0) {
	    log = "Could not connect.";
	    BOOST_LOG_TRIVIAL(debug) << log;
	    return false;
	} else {
	    BOOST_LOG_TRIVIAL(debug) << "Connected.";
	    SensorUsb->save(SensorUsb->save());
	    // Dealing with saving:

	}
    } else if (al1 == "disconnect") {

	SensorUsb->getSensor()->stop();

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
		SensorUsb->loadPath(it->second.data());
	    }
	    it = vals.find("fileBegin");
	    if(it != vals.not_found()) {
		SensorUsb->beginFile(boost::lexical_cast<int>(it->second.data()));
	    }
	    it = vals.find("fileEnd");
	    if(it != vals.not_found()) {
		SensorUsb->endFile(boost::lexical_cast<int>(it->second.data()));
	    }

	    if(al2 == "start") {
		SensorUsb->save(false);
		SensorUsb->playback(true);
		//TODO fileBegin should define this
		SensorUsb->cnt(1);
	    } else if(al2 == "stop") {
		SensorUsb->playback(false);
	    } else {
		jOutput.add("status", "failed");
		jOutput.add("reason", "Unknown action.");
		ss.clear();
		boost::property_tree::json_parser::write_json(ss,jOutput);
		log = ss.str();
		return false;
	    }
	}

	jOutput.add("loadPath",SensorUsb->loadPath());
	jOutput.add("fileBegin",SensorUsb->beginFile());
	jOutput.add("fileEnd",SensorUsb->endFile());
	jOutput.add("playBack",SensorUsb->playback());
	jOutput.add("type", SensorUsb->type());

    } else if (al1 == "control") {

	if (SensorUsb->getSensor()->isConnected()) {

	    if (!action.req->content.empty()) {
		std::stringstream ssInput(action.req->content);
		boost::property_tree::ptree vals;

		BOOST_LOG_TRIVIAL(debug) << "Content: " << input;
		boost::property_tree::json_parser::read_json(ssInput,vals);
//		boost::property_tree::ptree::const_assoc_iterator it = vals.find("it");
//		if (it != vals.not_found()) {
//		    SensorUsb->getSensor()->getCamera().setIntegrationTime(boost::lexical_cast<int>(it->second.data()));
//		}
//		it = vals.find("fr");
//		if(it != vals.not_found()) {
//		    SensorUsb->getSensor()->getCamera().setFrameRate(boost::lexical_cast<float>(it->second.data()));
//		}
//		it = vals.find("fr");
//		if(it != vals.not_found()) {
//		    SensorUsb->getSensor()->getCamera().setModulationFrequency(boost::lexical_cast<int>(it->second.data()));
//		}
	    }
//	    jOutput.add("it",SensorUsb->getSensor()->getCamera().getIntegrationTime());
//	    jOutput.add("mf",SensorUsb->getSensor()->getCamera().getModulationFrequency());
//	    jOutput.add("fr",SensorUsb->getSensor()->getCamera().getFrameRate());

	}
    } else if (al1 == "save") {
	if (action._items.size() >= 4) {
	    al2 = action._items[3];
	    if (SensorUsb->playback())
		SensorUsb->save(false);
	    else {
		std::stringstream ssInput(input);
		boost::property_tree::ptree vals;

		BOOST_LOG_TRIVIAL(debug) << "Content: " << input;
		boost::property_tree::json_parser::read_json(ssInput,vals);

		boost::property_tree::ptree::const_assoc_iterator it = vals.find("savePath");
		if (it != vals.not_found()) {
		    SensorUsb->savePath(it->second.data());
		}
		it = vals.find("saveFolder");
		if(it != vals.not_found()) {
		    SensorUsb->saveFolder(it->second.data());
		}
		it = vals.find("timeStamped");
		if(it != vals.not_found()) {
		    SensorUsb->timeStamped(true);
		} else
		    SensorUsb->timeStamped(false);
		if(al2 == "start") {
		    if (!SensorUsb->playback()) {
			SensorUsb->save(true);
		    }
		} else if(al2 == "stop") {
		    SensorUsb->save(false);
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
	jOutput.add("save",SensorUsb->save());
	jOutput.add("savePath",SensorUsb->getSavePath());
	jOutput.add("saveFolder",SensorUsb->saveFolder());
	jOutput.add("timeStamped",SensorUsb->timeStamped());

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
