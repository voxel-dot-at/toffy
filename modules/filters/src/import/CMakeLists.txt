add_library(toffy_import OBJECT 
    init.cpp
    dataimporter.cpp
    importYaml.cpp
    )

target_link_libraries(  toffy_import toffy_core ${LIBS} )
