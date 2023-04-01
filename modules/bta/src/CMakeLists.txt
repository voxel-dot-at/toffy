add_library(toffy_bta OBJECT 
    bta.cpp
    BtaWrapper.cpp
    csv_source.cpp
    initPlugin.cpp
    )
target_link_libraries( toffy_bta toffy_core ${LIBS} )
