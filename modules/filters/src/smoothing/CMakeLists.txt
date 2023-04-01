add_library(toffy_smoothing OBJECT 
    average.cpp
    bilateral.cpp
    kalmanaverage.cpp
    )

target_link_libraries(  toffy_smoothing toffy_core ${LIBS} )
