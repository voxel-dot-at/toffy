
#include <iostream>
#include <toffy/bta/bta.hpp>
// #include <toffy/bta/bta_cb.hpp>
#include <toffy/bta/initPlugin.hpp>


// #include <toffy_bta/toffy_bta_config.h>

using namespace toffy;

void toffy::bta::init(toffy::FilterFactory *ff) 
{
    BOOST_LOG_TRIVIAL(info) << "BTA:: HERE INIT";
    ff->registerCreator(toffy::capturers::Bta::id_name, &toffy::capturers::Bta::creator);
    // callback-based variant - work in progress
    // ff->registerCreator(toffy::capturers::BtaCb::id_name, &toffy::capturers::BtaCb::creator);
    return;
}
