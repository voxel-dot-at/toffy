

add_executable(test_cond test_cond.cpp)
target_link_libraries(test_cond toffy)


add_executable(test_logging test_logging.cpp ../modules/core/src/logging.cpp)
target_link_libraries(test_logging )

