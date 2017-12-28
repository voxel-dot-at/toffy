#pragma once

#include <toffy/web/filterController.hpp>
#include <toffy/web/requestAction.hpp>

namespace toffy {
namespace control {

/**
 * @brief Extended controller to handle multiples sensor2d filter at the same time
 * @ingroup Control
 *
 * Apply the Action on all 2d filters declared in the filterBanks
 */
class Sensor2dGroupController : public FilterController {
    std::vector<toffy::Filter *> _filterList;
public:

    Sensor2dGroupController() {}
    Sensor2dGroupController(std::vector<toffy::Filter *> vec) : _filterList(vec) {}
    virtual ~Sensor2dGroupController() {}

    virtual bool doAction(Action &action, std::string &log);

private:
};

} // namespace controller
} // namespace toffy
