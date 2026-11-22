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

add_executable(test_logging test_logging.cpp)
target_link_libraries(test_logging toffy ${LOG4CPLUS_LIBRARIES} GTest::gtest GTest::gtest_main)
add_test(NAME logging COMMAND test_logging)

# P2-17: the toffy::log abstraction on its own, naming no backend -- this file
# has to pass unchanged when src/logging.cpp is swapped to log4cplus.
add_executable(test_logging_abstraction test_logging_abstraction.cpp)
target_link_libraries(test_logging_abstraction toffy GTest::gtest GTest::gtest_main)
add_test(NAME logging_abstraction COMMAND test_logging_abstraction)

# P3-9: asserts that a filter's body actually ran, for both override shapes.
# The gate P2-12 needs - see the header comment in the source.
add_executable(test_filter_overloads test_filter_overloads.cpp)
target_link_libraries(test_filter_overloads toffy GTest::gtest GTest::gtest_main)
add_test(NAME filter_overloads COMMAND test_filter_overloads)

# A24: CSVSource's frame counter / sequence flag. The fixture directory is
# passed as a compile definition rather than argv, because this target links
# gtest_main and InitGoogleTest would swallow an argument we cannot read from
# a TEST body. test_cond takes its fixture in argv precisely because it has no
# gtest_main.
add_executable(test_csv_source test_csv_source.cpp)
target_link_libraries(test_csv_source toffy GTest::gtest GTest::gtest_main)
target_compile_definitions(test_csv_source
    PRIVATE TOFFY_TEST_CSV_DIR="${CMAKE_CURRENT_SOURCE_DIR}/xml/csv")
add_test(NAME csv_source COMMAND test_csv_source)

# P3-6 (second half): ExportCSV used options/pattern as a printf format, the
# same finding N5 defect CSVSource had. Writes into gtest's temp dir only.
add_executable(test_exportcsv test_exportcsv.cpp)
target_link_libraries(test_exportcsv toffy GTest::gtest GTest::gtest_main)
add_test(NAME exportcsv COMMAND test_exportcsv)

# N4 / P3-5: ObjectTrack passed options/script to system(), so a config file
# could run a shell command. Writes into gtest's temp dir only, and the scripts
# it installs write there too - nothing is executed from the source tree.
add_executable(test_objecttrack test_objecttrack.cpp)
target_link_libraries(test_objecttrack toffy GTest::gtest GTest::gtest_main)
add_test(NAME objecttrack COMMAND test_objecttrack)

# P2-5: who owns a Filter. Counts Filter destructors, which is the only way to
# see the leak (a renamed filter was never released) and the use-after-free (a
# filter in two banks was destroyed by the first one) that the raw-pointer
# arrangement produced. The double free and the read-after-free are also visible
# to the ASan job; the leaks are visible to its leak check.
add_executable(test_ownership test_ownership.cpp)
target_link_libraries(test_ownership toffy GTest::gtest GTest::gtest_main)
add_test(NAME ownership COMMAND test_ownership)

# P2-7: the threading. FilterThread's start/stop/join states, the lane queue's
# condition variables, ParallelFilter's lanes without a barrier, and the
# frame-arrival signal on FilterBank. Several of these hang rather than fail on
# the code they replace, so each blocking wait in the file is bounded and reports
# instead of wedging the suite; the frame leak is a LeakSanitizer subject and is
# checked by the sanitizers job.
add_executable(test_filterthread test_filterthread.cpp)
target_link_libraries(test_filterthread toffy GTest::gtest GTest::gtest_main)
add_test(NAME filterthread COMMAND test_filterthread)

# P2-4: the factory's built-in types are registrations, not branches. The set is
# the assertion -- a dropped type name fails in the field, from a config file, at
# run time, and nothing in a compile or a link notices. `EveryExpectedTypeIsA-
# RegisteredCreator` is the half that is red while the chain is still in place;
# see the header comment in the source for what the set was measured against and
# for the three names that are deliberately still unreachable.
add_executable(test_filter_registry test_filter_registry.cpp)
target_link_libraries(test_filter_registry toffy GTest::gtest GTest::gtest_main)
add_test(NAME filter_registry COMMAND test_filter_registry)

# P2-9: modules/core must not reach up into the viewers module. Core owns the
# HighGUI-pump type name now; this test pins that copy against ImageView::id_name
# so the duplication cannot drift. See tests/test_layering.cpp and the matching
# hygiene gate in .github/workflows/ci.yml.
add_executable(test_layering test_layering.cpp)
target_link_libraries(test_layering toffy GTest::gtest GTest::gtest_main)
add_test(NAME layering COMMAND test_layering)

# P2-9: the bta* slot-name constants used to be defined in an installed header, so
# each TU got its own copy. Two sources on purpose -- the single-instance property
# is only observable across translation units. See tests/test_btaframe_constants.cpp.
add_executable(test_btaframe_constants test_btaframe_constants.cpp test_btaframe_constants_addr.cpp)
target_link_libraries(test_btaframe_constants toffy GTest::gtest GTest::gtest_main)
add_test(NAME btaframe_constants COMMAND test_btaframe_constants)

# P2-10: three plugin loaders with three failure modes. This test pins the
# consolidated behaviour -- a missing <plugins> node is not an error, and all
# three look at the same node. Needs log4cplus directly: it captures log records
# to observe whether a loader actually visited the node.
# A real plug-in, built as a MODULE, so the loader test has something that
# actually loads. Before this the plug-in path was only ever exercised with a
# library that does not exist -- which covers the failure branch and nothing else.
add_library(toffy_test_plugin MODULE plugin_fixture.cpp)
set_target_properties(toffy_test_plugin PROPERTIES PREFIX "" SUFFIX ".so")
target_link_libraries(toffy_test_plugin toffy)

add_executable(test_plugin_loading test_plugin_loading.cpp)
add_dependencies(test_plugin_loading toffy_test_plugin)
target_compile_definitions(test_plugin_loading PRIVATE
  TOFFY_TEST_PLUGIN="$<TARGET_FILE:toffy_test_plugin>")
target_link_libraries(test_plugin_loading toffy ${LOG4CPLUS_LIBRARIES} GTest::gtest GTest::gtest_main)
add_test(NAME plugin_loading COMMAND test_plugin_loading)
