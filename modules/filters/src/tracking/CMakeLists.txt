
message("OPENCV TRACKING? " ${OPENCV_TRACKING_FOUND})
if ( ${OPENCV_TRACKING_FOUND} )
    set(TRACK_SRCS 
        cvTracker.cpp
    )
else() 
    message("opencv tracking module not found - will NOT build cvTracker")
    set(TRACK_SRCS "" )
endif()

add_library(toffy_tracking OBJECT 
    tracker.cpp
    ${TRACK_SRCS}
    )
