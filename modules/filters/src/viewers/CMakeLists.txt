if ( ${PCL_VISUALIZATION_FOUND} )
    set(PCL_SRCS 
        cloudviewpcl.cpp
    )
    add_definitions(-DPCL_VIZ=1)
else() 
    message("pcl viz module not found - will NOT build cloudviewpcl")
    set(PCL_SRCS "" )
endif()

if ( ${OPENCV_VIZ_FOUND} )
    set(VIZ_SRCS 
        cloudviewopencv.cpp
    )
    add_definitions(-DOPENCV_VIZ=1)
else() 
    message("opencv viz module not found - will NOT build cloudviewopencv")
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
    init.cpp
    )

# target_link_libraries(  toffy_viewers toffy_core ${LIBS} )
