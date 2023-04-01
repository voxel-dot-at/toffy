
include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/../include/" )

add_subdirectory(base)

add_subdirectory(capture)

add_subdirectory(detection)

add_subdirectory(import)

add_subdirectory(3d)

add_subdirectory(reproject)

add_subdirectory(smoothing)

add_subdirectory(tracking)

add_subdirectory(viewers)
