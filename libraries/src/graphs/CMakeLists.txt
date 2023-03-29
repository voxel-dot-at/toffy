add_library(toffy_lib_graphs STATIC 
    boundingBox.cpp
    graph.cpp 
    raster.cpp
    )

target_link_libraries(  toffy_lib_graphs ${LIBS} )
