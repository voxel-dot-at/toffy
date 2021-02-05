
#include <iostream>
#include <toffy/bta/bta.hpp>
// #include <toffy/bta/bta_cb.hpp>
#include <toffy/bta/initPlugin.hpp>


// #include <toffy_bta/toffy_bta_config.h>

#ifdef WITH_CONTROL
    #include <toffy_web/plugins.hpp>
    #include <toffy_web/btaController.hpp>
    #include <toffy_web/btaGroupController.hpp>
#endif

using namespace toffy;

void toffy::bta::init(toffy::FilterFactory *ff) 
{
    BOOST_LOG_TRIVIAL(info) << "BTA:: HERE INIT";
    ff->registerCreator(toffy::capturers::Bta::id_name, &toffy::capturers::Bta::creator);
    // callback-based variant - work in progress
    // ff->registerCreator(toffy::capturers::BtaCb::id_name, &toffy::capturers::BtaCb::creator);

#ifdef WITH_CONTROL
    control::ControllerFactory* cf = control::ControllerFactory::getInstance();
    initUI(cf);
#endif
    return;
}

#ifdef WITH_CONTROL
void toffy::bta::initUI(control::ControllerFactory *cf) 
{
    BOOST_LOG_TRIVIAL(info) << "BTA:: HERE INIT UI";
    cf->registerCreator(toffy::capturers::Bta::id_name, &toffy::control::BtaController::creator);
    cf->registerCreator(toffy::capturers::Bta::id_name+"Group", &toffy::control::BtaGroupController::creator);
    return;
}
#endif
