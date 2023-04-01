add_library(toffy_detection OBJECT 
    blobs.cpp
    blobsDetector.cpp
    detectedObject.cpp
    mask.cpp
    objectTrack.cpp
    simpleBlobs.cpp
    squareDetect.cpp
    )

target_link_libraries(  toffy_detection toffy_core ${LIBS} )
