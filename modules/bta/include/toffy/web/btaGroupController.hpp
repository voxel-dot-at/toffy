#pragma once

#include <toffy_web/filterController.hpp>
#include <toffy_web/requestAction.hpp>

namespace toffy {
namespace control {

/**
 * @brief Extended controller to handle multiples bta filter at the same time
 * @ingroup Control
 *
 * Apply the Action on all bta filters declared in the filterBanks
 */
class BtaGroupController : public FilterController {
public:

    BtaGroupController() {}

    /**
     * @brief Alternative constructor with a list of filters
     * @param vec
     */
    BtaGroupController(std::vector<toffy::Filter *> vec) : _filterList(vec) {}

    virtual ~BtaGroupController() {}

    virtual bool doAction(Action &action, std::string &log);

    static FilterController* creator() {return new BtaGroupController();}

private:
    std::vector<toffy::Filter *> _filterList;
};

} // namespace controller
} // namespace toffy
