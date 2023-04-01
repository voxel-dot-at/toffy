
include_directories(../server/uri-grammar)

add_library(toffy_control SHARED 
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
target_link_libraries(toffy_control toffy_control_server toffy ${LIBS})
