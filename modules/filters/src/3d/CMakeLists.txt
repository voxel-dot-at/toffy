
message (STATUS "PCL_VIZ in 3D: " ${PCL_VISUALIZATION_FOUND})

# if() takes variable *names*; ${PCL_FOUND} expanded to nothing when PCL was off.
if (PCL_FOUND)
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

# The target is only created when there are sources: CMake rejects an OBJECT
# library with an empty source list ("No SOURCES given to target: toffy_3d"), which
# is what made -DWITHOUT_PCL=ON fail to configure. Consumers test PCL_FOUND the same
# way rather than naming $<TARGET_OBJECTS:toffy_3d> unconditionally.
if (PCL_SRCS)
    add_library(toffy_3d OBJECT ${PCL_SRCS}
        )
endif()

# target_link_libraries(  toffy_3d toffy_core ${LIBS} )
