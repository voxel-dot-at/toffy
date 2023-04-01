add_library(toffy_reproject OBJECT 
    reprojectopencv.cpp
    reprojectpcl.cpp
    )

target_link_libraries(  toffy_reproject toffy_core ${LIBS} )
