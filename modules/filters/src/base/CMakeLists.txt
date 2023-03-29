add_library(toffy_base STATIC 
    amplitudeRange.cpp
    backgroundsubs.cpp
    cond.cpp
    distAmpl.cpp
    focus.cpp
    nop.cpp
    offset.cpp
    ofsCorr.cpp
    polar2cart.cpp
    range.cpp
    rectify.cpp
    roi.cpp
    init.cpp
    )

target_link_libraries(  toffy_base toffy_core ${LIBS} )
