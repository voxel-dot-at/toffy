
if ( ${PCL_VISUALIZATION_FOUND} )
    set(PCL_SRCS 
        cloudviewpcl.cpp
    )
else() 
    messate("pcl viz module not found - will NOT build cloudviewpcl")
    set(PCL_SRCS "" )
endif()

message("OPENCV VIZ? " ${OPENCV_VIZ_FOUND})
if ( ${OPENCV_VIZ_FOUND} )
    set(VIZ_SRCS 
        cloudviewopencv.cpp
    )
else() 
    messate("opencv viz module not found - will NOT build cloudviewopencv")
    set(VIZ_SRCS "" )
endif()

add_library(toffy_viewers OBJECT 
    colorize.cpp
    exportcloud.cpp
    exportcsv.cpp
    imageview.cpp
    videoout.cpp
    ${VIZ_SRCS}
    ${PCL_SRCS}
    )

# target_link_libraries(  toffy_viewers toffy_core ${LIBS} )
