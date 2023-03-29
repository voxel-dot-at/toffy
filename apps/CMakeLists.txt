
#Include configs in demo app
file(GLOB_RECURSE configs configs/*.xml )
add_executable (toffyRunner main.cpp ${configs})

target_compile_definitions(toffyRunner PUBLIC ${DEFINITIONS})
# toffy_DEBUG only get set alone
target_compile_definitions(toffyRunner PUBLIC "${PROJECT_NAME}_DEBUG=$<CONFIG:Debug>")
target_link_libraries(toffyRunner ${LIBS} dl ${PROJECT_NAME} )

list(APPEND binaries ${CMAKE_CURRENT_BINARY_DIR}/toffyRunner)


add_executable (tst_pb tst_pb.cpp)
target_compile_definitions(tst_pb PUBLIC ${DEFINITIONS} PLAT_LINUX "${PROJECT_NAME}_DEBUG=$<CONFIG:Debug>")
target_link_libraries(tst_pb ${LIBS} dl ${PROJECT_NAME} )
