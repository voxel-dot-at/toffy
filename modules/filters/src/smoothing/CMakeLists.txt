if ( ${OPENCV_TRACKING_FOUND} )
    set(TRACK_SRCS 
        kalmanaverage.cpp
    )
    add_definitions(-DOPENCV_TRACKING=1)
else() 
    message("opencv tracking module not found - will NOT build kalmanaverage")
    set(TRACK_SRCS "" )
endif()

add_library(toffy_smoothing OBJECT 
    average.cpp
    bilateral.cpp
    ${TRACK_SRCS}
    )

target_link_libraries(  toffy_smoothing toffy_core ${LIBS} )
