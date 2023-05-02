
if ( ${PCL_VISUALIZATION_FOUND} )
    set(PCL_SRCS 
        cloudviewpcl.cpp
    )
else() 
    set(PCL_SRCS "" )
endif()

add_library(toffy_viewers OBJECT 
    cloudviewopencv.cpp
    colorize.cpp
    exportcloud.cpp
    exportcsv.cpp
    imageview.cpp
    videoout.cpp
    ${PCL_SRCS}
    )

# target_link_libraries(  toffy_viewers toffy_core ${LIBS} )
