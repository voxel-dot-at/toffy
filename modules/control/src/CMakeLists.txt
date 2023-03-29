
include_directories(../server/uri-grammar)

add_library(toffy_control STATIC 
    capturersController.cpp
    controllerFactory.cpp
    filterController.cpp
    requestAction.cpp
    requestController.cpp
    sensor2dController.cpp
    sensor2dGroupController.cpp
    sensorusbController.cpp
    webcontroller.cpp
    )
