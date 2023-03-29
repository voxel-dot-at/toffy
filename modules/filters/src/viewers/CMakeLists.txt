add_library(toffy_viewers STATIC 
    cloudviewopencv.cpp
    cloudviewpcl.cpp
    colorize.cpp
    exportcloud.cpp
    exportcsv.cpp
    imageview.cpp
    videoout.cpp
    )

target_link_libraries(  toffy_viewers toffy_core ${LIBS} )
