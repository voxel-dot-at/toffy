add_library(toffy_import STATIC 
    dataimporter.cpp
    )

target_link_libraries(  toffy_import toffy_core ${LIBS} )
