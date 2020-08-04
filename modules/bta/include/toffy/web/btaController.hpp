#pragma once

#include <toffy_web/filterController.hpp>
#include <toffy_web/action.hpp>
//#include <toffy/bta.hpp>


namespace toffy {
namespace control {

/**
 * @brief Web controller class for handle the specific operations on the
 * bta filter.
 */
class BtaController : public FilterController {
public:

    BtaController() {}

    /**
     * @brief Constructor for filling the base class
     * @param f Filter in toffy to be handled
     */
    BtaController(toffy::Filter *f) : FilterController(f) {}

    virtual ~BtaController() {}
    virtual bool doAction(Action &action, std::string &log);

    static FilterController* creator() {return new BtaController();}
private:
};

} // namespace controller
} // namespace toffy

