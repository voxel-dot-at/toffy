add_library(toffy_reproject STATIC 
    reprojectopencv.cpp
    reprojectpcl.cpp
    )

target_link_libraries(  toffy_reproject toffy_core ${LIBS} )
