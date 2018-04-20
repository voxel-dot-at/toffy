#pragma once

#include <toffy/web/filterController.hpp>


namespace toffy {
namespace control {

class SensorUSBController : public FilterController {

public:

    SensorUSBController() {}
    SensorUSBController(toffy::Filter *f) : FilterController(f) {}
    virtual ~SensorUSBController() {}
    virtual bool doAction(Action &action, std::string &log);
private:
};

} // namespace controller
} // namespace toffy

