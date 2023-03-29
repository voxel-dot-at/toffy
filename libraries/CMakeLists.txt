add_subdirectory(src)

add_library(toffy_libs STATIC init.cpp)
target_link_libraries(toffy_libs toffy_lib_common toffy_lib_tracer toffy_lib_graphs toffy_lib_skel)
install (TARGETS toffy_libs
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
