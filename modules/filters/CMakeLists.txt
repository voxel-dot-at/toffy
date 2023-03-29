
add_subdirectory(src)

add_library(toffy_filters STATIC init.cpp )
target_link_libraries(toffy_filters PUBLIC toffy_base toffy_detection
    toffy_capture toffy_3d toffy_import
    toffy_reproject toffy_smoothing 
    toffy_tracking  toffy_viewers )
