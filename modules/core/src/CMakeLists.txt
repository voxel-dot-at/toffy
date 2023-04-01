add_library(toffy_core OBJECT 
    controller.cpp
    event.cpp
    filter.cpp
    filterbank.cpp
    filterfactory.cpp
    filterThread.cpp
    frame.cpp
    mux.cpp
    parallelFilter.cpp
    player.cpp
    )

target_link_libraries(  toffy_core ${LIBS} )
