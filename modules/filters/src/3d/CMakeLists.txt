add_library(toffy_3d OBJECT 
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

# target_link_libraries(  toffy_3d toffy_core ${LIBS} )
