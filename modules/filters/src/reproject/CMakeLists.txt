# reprojectpcl.cpp includes pcl headers, so it is only built when PCL is enabled.
# filterfactory.cpp already guards both the include and the "reprojectpcl" branch of
# createFilter() with #if defined(PCL_FOUND), so no registration change is needed
# here -- the target keeps reprojectopencv.cpp and so is never empty.
if (PCL_FOUND)
    set(PCL_REPROJ_SRCS reprojectpcl.cpp)
else()
    set(PCL_REPROJ_SRCS "")
endif()

add_library(toffy_reproject OBJECT 
    reprojectopencv.cpp
    ${PCL_REPROJ_SRCS}
    )

target_link_libraries(  toffy_reproject toffy_core ${LIBS} )
