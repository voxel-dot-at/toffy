add_library(toffy_lib_common STATIC 
    filenodehelper.cpp
    )

target_link_libraries(  toffy_lib_common ${LIBS} )
