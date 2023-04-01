add_library(toffy_capture OBJECT 
    capturerFilter.cpp
    )

target_link_libraries(  toffy_capture toffy_core ${LIBS} )
