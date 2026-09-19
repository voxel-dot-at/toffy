
add_subdirectory(src)

# toffy_3d is only created when PCL is enabled, so its objects are referenced
# conditionally -- naming $<TARGET_OBJECTS:toffy_3d> unconditionally would fail to
# generate when the target does not exist.
set(TOFFY_3D_OBJ "")
if (PCL_FOUND)
    set(TOFFY_3D_OBJ $<TARGET_OBJECTS:toffy_3d>)
endif()

add_library(toffy_filters OBJECT 
    $<TARGET_OBJECTS:toffy_base> 
    ${TOFFY_3D_OBJ} 
    $<TARGET_OBJECTS:toffy_capture> 
    $<TARGET_OBJECTS:toffy_detection> 
    $<TARGET_OBJECTS:toffy_import> 
    $<TARGET_OBJECTS:toffy_reproject> 
    $<TARGET_OBJECTS:toffy_smoothing> 
    $<TARGET_OBJECTS:toffy_tracking> 
    $<TARGET_OBJECTS:toffy_viewers> 
    )

install(DIRECTORY "include/" DESTINATION "include/")
