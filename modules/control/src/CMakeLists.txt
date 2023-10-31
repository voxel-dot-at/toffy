
include_directories(../server/uri-grammar)

add_library(toffy_web_objs OBJECT 
    capturersController.cpp
    controllerFactory.cpp
    filterController.cpp
    requestAction.cpp
    requestController.cpp
    sensor2dController.cpp
    sensor2dGroupController.cpp
    sensorusbController.cpp
    webcontroller.cpp
    $<TARGET_OBJECTS:toffy_web_server> 
    )

add_library(toffy_web SHARED 
    capturersController.cpp
    controllerFactory.cpp
    filterController.cpp
    requestAction.cpp
    requestController.cpp
    sensor2dController.cpp
    sensor2dGroupController.cpp
    sensorusbController.cpp
    webcontroller.cpp
    $<TARGET_OBJECTS:toffy_web_server> 
    )

target_link_libraries(toffy_web toffy ${LIBS})
set_property(TARGET toffy_web PROPERTY VERSION ${PROJECT_VERSION})
set_property(TARGET toffy_web PROPERTY SOVERSION ${PROJECT_VERSION})
install (TARGETS toffy_web
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )
