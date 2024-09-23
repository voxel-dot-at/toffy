add_library(toffy_import OBJECT 
    dataimporter.cpp
    importYaml.cpp
    )

target_link_libraries(  toffy_import toffy_core ${LIBS} )
