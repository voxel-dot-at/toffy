add_library(toffy_lib_skel OBJECT

    depthSkeletonizer.cpp          
    erodeSkeletonizer.cpp
    k3mSkeletonizer.cpp
    k3mppSkeletonizer.cpp
    k3mUpDownSkeletonizer.cpp
    skeletonizerFactory.cpp
    skeletonizer.cpp
    thickSkeletonizer.cpp
    )

target_link_libraries(  toffy_lib_skel ${LIBS} )
