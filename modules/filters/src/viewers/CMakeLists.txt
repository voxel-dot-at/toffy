# if() takes variable *names*: expanding ${PCL_VISUALIZATION_FOUND} here left it
# empty when PCL is disabled, so CMake parsed "if( AND OFF)" and aborted with
# "Unknown arguments specified". That made -DWITHOUT_PCL=ON unconfigurable, which
# is why the unguarded pcl typedefs in frame.hpp went unnoticed.
if (PCL_VISUALIZATION_FOUND AND WITH_PCL_CLOUDVIEW)
    set(PCL_SRCS 
        cloudviewpcl.cpp
    )
    add_definitions(-DPCL_VIZ=1)
else() 
    message("pcl viz module not found - will NOT build cloudviewpcl")
    set(PCL_SRCS "" )
endif()

# Same fix as above: latent, because it only breaks when OpenCV's viz component is
# absent -- currently it is installed, so the expansion happens to be non-empty.
if (OPENCV_VIZ_FOUND AND WITH_VISUALIZATION)
    set(VIZ_SRCS 
        cloudviewopencv.cpp
    )
    add_definitions(-DOPENCV_VIZ=1)
else()
    message("opencv viz module not found - will NOT build cloudviewopencv")
    set(VIZ_SRCS "" )
endif()

# exportcloud needs PCL itself -- it holds a pcl::PCDWriter by value -- not just
# the pcl visualization component, so it is gated on PCL_FOUND rather than on
# WITH_PCL_CLOUDVIEW. Its class declaration, factory function and registration are
# guarded the same way, so the three stay in step.
if (PCL_FOUND)
    set(PCL_IO_SRCS exportcloud.cpp)
else()
    set(PCL_IO_SRCS "")
endif()

add_library(toffy_viewers OBJECT 
    colorize.cpp
    exportcsv.cpp
    exportYaml.cpp
    imageview.cpp
    videoout.cpp
    ${VIZ_SRCS}
    ${PCL_SRCS}
    ${PCL_IO_SRCS}
    init.cpp
    )

# target_link_libraries(  toffy_viewers toffy_core ${LIBS} )
