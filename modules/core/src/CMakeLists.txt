add_library(toffy_core OBJECT 
    controller.cpp
    filter.cpp
    filterbank.cpp
    filterfactory.cpp
    filterThread.cpp
    frame.cpp
    mux.cpp
    parallelFilter.cpp
    player.cpp
    )

target_link_libraries(  toffy_core ${LIBS} )

# P3-4: modules/core is warning-free under the project's -Wall -Wextra, so hold
# the line here and only here. The rest of the tree still emits ~28 warnings per
# build and is deliberately not gated: tools/warning_report.sh prints the count per
# area on every CI build (P3-11) and fails only on modules/core.
#
# Scoped to this target, not add_compile_options(), so it cannot leak into a
# sibling target; guarded on the compiler id because -Werror is /WX on MSVC.
# CORE_WERROR=OFF exists for downstream builds on a compiler this tree has not
# been measured with - it is an escape hatch, not a supported mode.
option(CORE_WERROR "Treat warnings in modules/core as errors" ON)
if (CORE_WERROR AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(toffy_core PRIVATE -Werror)
endif()
