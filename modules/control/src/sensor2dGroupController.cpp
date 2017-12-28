#include <toffy/web/sensor2dGroupController.hpp>
#include <toffy/web/sensor2dController.hpp>
#include <boost/log/trivial.hpp>
#include <boost/lexical_cast.hpp>
//#include <toffy/sensor2d.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace toffy {
namespace control {
using namespace std;

bool Sensor2dGroupController::doAction(Action &action, std::string &log) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    boost::property_tree::ptree jOutput;
    std::stringstream ss;
/*
    string name;
    if (action._items.size() > 4)
	name = action._items[3];
    BOOST_LOG_TRIVIAL(debug) << name;
    string input = action.req->content;
    if (name.empty()) {
	if (!input.empty()) {
	    BOOST_LOG_TRIVIAL(debug) << "Content: " << input;
	}
	BOOST_LOG_TRIVIAL(debug) << "_filterList.size(): " << _filterList.size();
	boost::property_tree::ptree sensor2ds;
	bool saveAll = true, playBackAll = true, connectedAll = true;
	for (size_t i = 0; i < _filterList.size(); i++) {
	    sensor2ds.add(_filterList[i]->name(),_filterList[i]->name());
	    if(!dynamic_cast<toffy::capturers::Sensor2*>(_filterList[i])->save())
		saveAll = false;
	    if(!dynamic_cast<toffy::capturers::Sensor2*>(_filterList[i])->playback())
		playBackAll = false;
	    if(!dynamic_cast<toffy::capturers::Sensor2*>(_filterList[i])->getSensor()->isConnected())
		connectedAll = false;
	}
	jOutput.add_child("filtergroup",sensor2ds);
	jOutput.add("saveAll",saveAll);
	jOutput.add("playBackAll",playBackAll);
	jOutput.add("connectedAll",connectedAll);

    } else if (name == "playBackAll") {
	if (action._items.size() < 5) {
	    jOutput.add("status","failed");
	    jOutput.add("reason","No action indicated");
	    boost::property_tree::json_parser::write_json(ss,jOutput);
	    log = ss.str();
	    return false;
	}
	toffy::capturers::Sensor2 *f;
	for (size_t i = 0; i < _filterList.size(); i++) {
	    f = dynamic_cast<toffy::capturers::Sensor2*>(_filterList[i]);
	    if (action._items[4] == "start") {
		f->save(false);
		f->playback(true);
	    } else if (action._items[4] == "stop") {
		f->playback(false);
	    } else  {
		jOutput.add("status","failed");
		jOutput.add("reason","Wrong json config value");
		boost::property_tree::json_parser::write_json(ss,jOutput);
		log = ss.str();
		return false;
	    }
	}

    } else if (name == "connectAll") {
	if (action._items.size() < 5) {
	    jOutput.add("status","failed");
	    jOutput.add("reason","No action indicated");
	    boost::property_tree::json_parser::write_json(ss,jOutput);
	    log = ss.str();
	    return false;
	}
	toffy::capturers::Sensor2 *f;
	for (size_t i = 0; i < _filterList.size(); i++) {
	    f = dynamic_cast<toffy::capturers::Sensor2*>(_filterList[i]);
	    if (action._items[4] == "start") {
		f->playback(false);
		if (f->getSensor()->start() < 0) {
		    jOutput.add("status","failed");
		    jOutput.add("reason","Could not start sensor " + f->name());
		    boost::property_tree::json_parser::write_json(ss,jOutput);
		    log = ss.str();
		    return false;
		} else {
		    BOOST_LOG_TRIVIAL(debug) << "Connected " << f->name();
		f->save(f->save());
		// Dealing with saving:
		}
	    } else if (action._items[4] == "stop") {
		f->getSensor()->stop();
	    } else  {
		jOutput.add("status","failed");
		jOutput.add("reason","Wrong json config value");
		boost::property_tree::json_parser::write_json(ss,jOutput);
		log = ss.str();
		return false;
	    }
	}
    } else if (name == "saveAll") {
	  BOOST_LOG_TRIVIAL(debug) << action._items[4];
	  BOOST_LOG_TRIVIAL(debug) << action._items.size();
	if (action._items.size() < 5) {
	    jOutput.add("status","failed");
	    jOutput.add("reason","No action indicated");
	    boost::property_tree::json_parser::write_json(ss,jOutput);
	    log = ss.str();
	    return false;
	}
	toffy::capturers::Sensor2 *f;
	for (size_t i = 0; i < _filterList.size(); i++) {
	    f = dynamic_cast<toffy::capturers::Sensor2*>(_filterList[i]);
	    if (action._items[4] == "start") {
		f->save(true);
	    } else if (action._items[4] == "stop") {
		f->save(false);

	    } else {
		jOutput.add("status","failed");
		jOutput.add("reason","Wrong json config value");
		boost::property_tree::json_parser::write_json(ss,jOutput);
		log = ss.str();
		return false;
	    }
	}

    } else  {
	jOutput.add("status","failed");
	jOutput.add("reason","Action unknown.");
	boost::property_tree::json_parser::write_json(ss,jOutput);
	log = ss.str();
	return false;
    }
*/
    jOutput.add("status","ok");
    boost::property_tree::json_parser::write_json(ss,jOutput);
    log = ss.str();
    return true;

}

} // namespace control
} // namespace l3vel
