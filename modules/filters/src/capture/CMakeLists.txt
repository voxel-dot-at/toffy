# csv_source.cpp moved here from modules/bta: it is a generic CSV playback
# capturer whose only dependencies are Boost and OpenCV -- no bta, no pcl. Living in
# the bta module meant it vanished from BTA-less builds even though it never needed
# the camera library, and filterfactory.cpp referenced it unconditionally.
add_library(toffy_capture OBJECT 
    capturerFilter.cpp
    csv_source.cpp
    )

target_link_libraries(  toffy_capture toffy_core ${LIBS} )
