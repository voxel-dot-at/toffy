add_library(toffy_capture STATIC 
    capturerFilter.cpp
    )

target_link_libraries(  toffy_capture toffy_core ${LIBS} )
