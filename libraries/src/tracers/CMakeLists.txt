add_library(toffy_lib_tracer OBJECT 
    pruner.cpp
    segmentTracer.cpp
    segments.cpp
    thickTracer8.cpp
    )

target_link_libraries(  toffy_lib_tracer ${LIBS} )
