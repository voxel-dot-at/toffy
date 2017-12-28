#pragma once

#include <toffy/web/filterController.hpp>
#include <toffy/web/requestAction.hpp>

namespace toffy {
namespace control {

/**
 * @brief Extended web controller that  allows to apply actions to all defined
 * capturers in the filterBanks.
 * @ingroup Control
 */
class CapturersController : public FilterController {
public:

    CapturersController() {}

    /**
     * @brief Alternative constructor with a list of filters
     * @param vec
     */
    CapturersController(std::vector<toffy::Filter *> vec) : _filterList(vec) {}

    virtual ~CapturersController() {}

    virtual bool doAction(Action &action, std::string &log);

private:
    std::vector<toffy::Filter *> _filterList;
};

} // namespace controller
} // namespace toffy
