
message (STATUS "PCL_VIZ in 3D: " ${PCL_VISUALIZATION_FOUND})

if ( ${PCL_FOUND} )
    set(PCL_SRCS 
        boundbox.cpp
        groundprojection.cpp
        merge.cpp
        muxMerge.cpp
        sampleConsensus.cpp
        split.cpp
        transform.cpp
        xyz2mat.cpp
        xyz2pcl.cpp
    )
else() 
    set(PCL_SRCS "" )
endif()

add_library(toffy_3d OBJECT ${PCL_SRCS}
    )

# target_link_libraries(  toffy_3d toffy_core ${LIBS} )
