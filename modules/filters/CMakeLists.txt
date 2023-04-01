
add_subdirectory(src)

add_library(toffy_filters OBJECT init.cpp 
    $<TARGET_OBJECTS:toffy_base> 
    $<TARGET_OBJECTS:toffy_3d> 
    $<TARGET_OBJECTS:toffy_capture> 
    $<TARGET_OBJECTS:toffy_detection> 
    $<TARGET_OBJECTS:toffy_import> 
    $<TARGET_OBJECTS:toffy_reproject> 
    $<TARGET_OBJECTS:toffy_smoothing> 
    $<TARGET_OBJECTS:toffy_tracking> 
    $<TARGET_OBJECTS:toffy_viewers> 
    )

install(DIRECTORY "include/" DESTINATION "include/")
