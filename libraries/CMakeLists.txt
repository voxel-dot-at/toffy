add_subdirectory(src)

add_library(toffy_libs OBJECT 
    $<TARGET_OBJECTS:toffy_lib_common> 
    $<TARGET_OBJECTS:toffy_lib_tracer> 
    $<TARGET_OBJECTS:toffy_lib_graphs> 
    $<TARGET_OBJECTS:toffy_lib_skel> 
    $<TARGET_OBJECTS:toffy_tracking> 
    init.cpp
    )

install(DIRECTORY "include/" DESTINATION "include/")
