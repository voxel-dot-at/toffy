
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/log/trivial.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <toffy/capture/capturerFilter.hpp>

#include <toffy/web/capturersController.hpp>

using namespace toffy;
using namespace toffy::control;
namespace fs = boost::filesystem;
using namespace std;

bool CapturersController::doAction(Action &action, std::string &log) {
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    boost::property_tree::ptree jOutput;
    std::stringstream ss;

    string name;
    BOOST_LOG_TRIVIAL(debug) << action._items.size();
    if (action._items.size() >= 4)
	name = action._items[3];
    BOOST_LOG_TRIVIAL(debug) << name;
    string input = action.req->content;
    if (name.empty()) {
	if (!input.empty()) {
	    BOOST_LOG_TRIVIAL(debug) << "Content: " << input;
	}
	BOOST_LOG_TRIVIAL(debug) << "_filterList.size(): " << _filterList.size();
	boost::property_tree::ptree btas;
	bool saveAll = true, playBackAll = true, connectedAll = true;
	for (size_t i = 0; i < _filterList.size(); i++) {
	    btas.add(_filterList[i]->name(),_filterList[i]->type());
	    if( !dynamic_cast<toffy::capturers::CapturerFilter*>(_filterList[i])->save())
		saveAll = false;
	    if( !dynamic_cast<toffy::capturers::CapturerFilter*>(_filterList[i])->playback())
		playBackAll = false;
	    if( !dynamic_cast<toffy::capturers::CapturerFilter*>(_filterList[i])->isConnected())
		connectedAll = false;
	}
	jOutput.add_child("filtergroup",btas);
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
	toffy::capturers::CapturerFilter *f;
	for (size_t i = 0; i < _filterList.size(); i++) {
	    f = dynamic_cast<toffy::capturers::CapturerFilter*>(_filterList[i]);
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
	toffy::capturers::CapturerFilter *f;
	for (size_t i = 0; i < _filterList.size(); i++) {
	    f = dynamic_cast<toffy::capturers::CapturerFilter*>(_filterList[i]);
	    if (action._items[4] == "start") {
		f->playback(false);
		if (f->connect() < 0) {
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
		f->disconnect();
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
	long int saveTimeStamp;
	#if defined(MSVC)
	using namespace boost::posix_time;
	ptime pt(second_clock::local_time());
	static ptime epoch(boost::gregorian::date(1970, 1, 1));
	time_duration diff(pt - epoch);
	saveTimeStamp= diff.ticks() / diff.ticks_per_second();
	#else
	time(&saveTimeStamp);
	#endif
	//string _strPath = savePath + string("/") + id() + string("/") + boost::lexical_cast<std::string>(_saveTimeStamp);

	toffy::capturers::CapturerFilter *f;
	for (size_t i = 0; i < _filterList.size(); i++) {
	    f = dynamic_cast<toffy::capturers::CapturerFilter*>(_filterList[i]);
	    if (action._items[4] == "start") {
		fs::path newPath(f->getSavePath());
		newPath /= boost::lexical_cast<std::string>(saveTimeStamp);
        //BOOST_LOG_TRIVIAL(debug) << newPath;
		fs::create_directories(fs::absolute(newPath));
		f->savePath(newPath.string());
		f->save(true);
	    } else if (action._items[4] == "stop") {
		fs::path newPath(f->getSavePath());
        //BOOST_LOG_TRIVIAL(debug) << newPath.parent_path();
		//newPath /= boost::lexical_cast<std::string>(saveTimeStamp);
		f->savePath(newPath.parent_path().string());
		f->save(false);

	    } else {
		jOutput.add("status","failed");
		jOutput.add("reason","Wrong json config value");
		boost::property_tree::json_parser::write_json(ss,jOutput);
		log = ss.str();
		return false;
	    }
	}
    } else if (name == "loadpaths") {
	if (!action.req->content.empty()) {
	    std::stringstream ssInput(action.req->content);
	    boost::property_tree::ptree vals;
	    BOOST_LOG_TRIVIAL(debug) << "name: " << name;
	    BOOST_LOG_TRIVIAL(debug) << "Content: " << input;
	    boost::property_tree::json_parser::read_json(ssInput,vals);
	    boost::property_tree::ptree::const_assoc_iterator it = vals.find("playBackPath");
	    if (it != vals.not_found()) {
		fs::path pbPaths(it->second.data());
        BOOST_LOG_TRIVIAL(debug) << pbPaths.string();
		if ( !fs::exists(pbPaths)) {
		    BOOST_LOG_TRIVIAL(warning) << "Given path " << pbPaths
					       << " does not exist.";
		    //return;
		}
		if ( !fs::is_directory(pbPaths)) {
		    BOOST_LOG_TRIVIAL(warning) << "Given path " << pbPaths
					       << " is not a directory."
					       << " Removing file name to get the path";
		    //fsPath = fsPath.parent_path();
		}

		fs::directory_iterator end_iter;

		//typedef std::multimap<int, fs::path> result_set_t;
		//result_set_t result_set;
		toffy::capturers::CapturerFilter *f;
		size_t i = 0;
        BOOST_LOG_TRIVIAL(debug) << pbPaths.string();
		//for (size_t i = 0; i < _filterList.size(); i++) {
		for( fs::directory_iterator dir_iter(pbPaths); dir_iter != end_iter ; ++dir_iter, i++) {
            BOOST_LOG_TRIVIAL(debug) << dir_iter->path().string();
		    if(fs::is_directory(dir_iter->path())) {
			for (size_t i = 0; i < _filterList.size(); i++) {
			    f = dynamic_cast<toffy::capturers::CapturerFilter*>(_filterList[i]);
			    if (dir_iter->path().string().find(f->name()) != string::npos)
				f->loadPath(dir_iter->path().string());
			}
			//f = dynamic_cast<toffy::capturers::CapturerFilter*>(_filterList[i]);
			//f->loadPath(dir_iter->path().string());
		    }

		}
	    }
	}
	BOOST_LOG_TRIVIAL(debug) << "OUT";

    } else  {
	jOutput.add("status","failed");
	jOutput.add("reason","Action unknown.");
	boost::property_tree::json_parser::write_json(ss,jOutput);
	log = ss.str();
	return false;
    }

    jOutput.add("status","ok");
    boost::property_tree::json_parser::write_json(ss,jOutput);
    log = ss.str();
    return true;

}
