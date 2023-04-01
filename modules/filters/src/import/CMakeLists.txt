add_library(toffy_import OBJECT 
    dataimporter.cpp
    )

target_link_libraries(  toffy_import toffy_core ${LIBS} )
