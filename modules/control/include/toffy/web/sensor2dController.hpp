#pragma once

#include <toffy/web/filterController.hpp>

namespace toffy {
namespace control {

class Sensor2dController : public FilterController {
public:

    Sensor2dController() {}
    Sensor2dController(toffy::Filter *f) : FilterController(f) {}
    virtual ~Sensor2dController() {}
    virtual bool doAction(Action &action, std::string &log);
private:
};

} // namespace controller
} // namespace toffy
