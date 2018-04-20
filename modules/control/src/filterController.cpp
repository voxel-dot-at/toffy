#include <toffy/web/filterController.hpp>
#include <boost/log/trivial.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace toffy;
using namespace toffy::control;

bool FilterController::doAction(Action &action, std::string &log) {
  BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
  std::stringstream ss;
  boost::property_tree::ptree jOutput;

  std::string al1, al2; //Action levels defined /toffy/al1/al2/al3.....
  if (action._items.size() > 2)
    al1 = action._items[2];

  if (al1 == "options") {
      if (!action.req->content.empty()) {
          std::stringstream ssInput(action.req->content);
          boost::property_tree::ptree vals;

          boost::property_tree::json_parser::read_json(ssInput,vals);
          _f->updateConfig(vals);
        }
      //Get option values
      jOutput = _f->getConfig();
    }

  jOutput.add("type", _f->type());
  jOutput.add("status", "ok");
  boost::property_tree::json_parser::write_json(ss,jOutput);
  log = ss.str();
  return true;
}
