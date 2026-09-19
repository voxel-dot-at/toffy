# Tests for toffy.
#
# Registered with CTest (see enable_testing() in the top-level CMakeLists.txt).
# Run with:  cmake --build . && ctest --output-on-failure

find_package(GTest REQUIRED)

# ---------------------------------------------------------------------------
# Existing config-loading test. Previously a bare executable that nothing ran.
# ---------------------------------------------------------------------------
add_executable(test_cond test_cond.cpp)
target_link_libraries(test_cond toffy)
add_test(
    NAME cond_empty
    COMMAND test_cond ${CMAKE_CURRENT_SOURCE_DIR}/xml/cond_empty.xml
)

# ---------------------------------------------------------------------------
# Core unit tests
# ---------------------------------------------------------------------------
add_executable(test_frame test_frame.cpp)
target_link_libraries(test_frame toffy GTest::gtest GTest::gtest_main)
add_test(NAME frame COMMAND test_frame)

add_executable(test_filter test_filter.cpp)
target_link_libraries(test_filter toffy GTest::gtest GTest::gtest_main)
add_test(NAME filter COMMAND test_filter)

add_executable(test_filterfactory test_filterfactory.cpp)
target_link_libraries(test_filterfactory toffy GTest::gtest GTest::gtest_main)
add_test(NAME filterfactory COMMAND test_filterfactory)

add_executable(test_controller test_controller.cpp)
target_link_libraries(test_controller toffy GTest::gtest GTest::gtest_main)
add_test(NAME controller COMMAND test_controller)

add_executable(test_player test_player.cpp)
target_link_libraries(test_player toffy GTest::gtest GTest::gtest_main)
add_test(NAME player COMMAND test_player)

add_executable(test_filterbank test_filterbank.cpp)
target_link_libraries(test_filterbank toffy GTest::gtest GTest::gtest_main)
add_test(NAME filterbank COMMAND test_filterbank)
