

#include <boost/log/trivial.hpp>

#include <toffy/web/controllerFactory.hpp>
//#include <toffy/web/btaController.hpp>
//#include <toffy/web/btaGroupController.hpp>
#include <toffy/web/capturersController.hpp>

#if defined(SENSOR2D)
#include <toffy/control/sensor2dController.hpp>
#include <toffy/control/sensorusbController.hpp>
#include <toffy/control/sensor2dGroupController.hpp>
#endif

// TODO Create a header with a list of filters

using namespace toffy;
using namespace toffy::control;
using namespace std;

ControllerFactory *ControllerFactory::_uniqueFactory;
boost::container::flat_map<std::string, FilterController *>
    ControllerFactory::_controllers;
ControllerFactory::ControllerFactory() {}

ControllerFactory *toffy::control::ControllerFactory::getInstance()
{
    if (_uniqueFactory == NULL) _uniqueFactory = new ControllerFactory();
    return _uniqueFactory;
}

ControllerFactory::~ControllerFactory()
{
    delete _uniqueFactory;
    // TODO do I have to explicily delete shared_ptr?
    _controllers.clear();
}

int ControllerFactory::deleteController(std::string name)
{
    /* boost::container::flat_map<std::string, Filter* >::iterator it;
     it = _filters.find(name);
     if (it != _filters.end() ) {
         delete it->second;
         _filters.erase(it);
         return 1;
     }*/
    return 0;
}

FilterController *ControllerFactory::getController(Filter *f)
{
    auto it = _controllers.find(f->id());
    if( it ==  _controllers.end() ) {
        return createController(f);
    } else {
        return it->second;
    }
}

FilterController *ControllerFactory::createController(Filter *f)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    FilterController *fc;

    CreateFilterControllerFn fn = _creators[f->type()];
    if (!fn) {
        fc = new FilterController(f);
    } else {
        fc = fn();
        fc->setFilter(f);
    }
    fc->setBaseController(_c);

    BOOST_LOG_TRIVIAL(debug)
        << "ControllerFactory::createController() for f->name(), id(): "
        << f->name() << "\t" << f->id();

    _controllers.insert(
        std::pair<std::string, FilterController *>(f->id(), fc));

    return fc;
}

int ControllerFactory::getControllersByType(
    const std::string &type, std::vector<FilterController *> &vec)
{
    BOOST_LOG_TRIVIAL(debug) << __FUNCTION__;
    for (boost::container::flat_map<std::string, FilterController *>::iterator
             it = _controllers.begin();
         it < _controllers.end(); it++) {
        // BOOST_LOG_TRIVIAL(debug) << type;
        // BOOST_LOG_TRIVIAL(debug) << (*it)->id();
        // if ((*it)->type() == "filterBank" ||
        //	(*it)->type() == "parallelFilter")
        //{
        //     ((FilterBank *)(*it))->getFiltersByType(type, vec);
        // }
        /*if (it->second->type() == type)
            vec.push_back(it->second);*/
    }
    return vec.size();
}

boost::container::flat_map<std::string, CreateFilterControllerFn>
    ControllerFactory::_creators;

void ControllerFactory::registerCreator(std::string name,
                                        CreateFilterControllerFn fn)
{
    _creators.insert(
        std::pair<std::string, CreateFilterControllerFn>(name, fn));
}

void ControllerFactory::unregisterCreator(std::string name)
{
    _creators.erase(name);
}
