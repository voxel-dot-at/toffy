add_library(toffy_tracking STATIC 
    tracker.cpp
    cvTracker.cpp
    )

target_link_libraries(  toffy_tracking toffy_core ${LIBS} )
