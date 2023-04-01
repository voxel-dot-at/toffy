#include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/../../include/" )
message("SRC ${CMAKE_CURRENT_SOURCE_DIR}/../include/")
add_library(toffy_viewers OBJECT 
    cloudviewopencv.cpp
    cloudviewpcl.cpp
    colorize.cpp
    exportcloud.cpp
    exportcsv.cpp
    imageview.cpp
    videoout.cpp
    )

# target_link_libraries(  toffy_viewers toffy_core ${LIBS} )
