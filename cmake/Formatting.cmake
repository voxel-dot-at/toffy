# P2-15 -- formatting targets.
#
# Scope is `modules/core`, deliberately, for the same reason `CORE_WERROR` (P3-4)
# is scoped there and not tree-wide: modules/core is warning-free under the
# project's -Wall -Wextra and is the part of the tree the cleanup programme is
# actually holding to a standard. The rest of the tree still emits ~23 warnings
# per build and is not gated. A formatting gate over everything would be a gate
# over code nobody has agreed a style for yet.
#
# Why a target and not just "run clang-format yourself": a `.clang-format` that
# nothing invokes is not a style, it is a suggestion nobody can check. Before this
# the file had sat at the repo root since 2024 applied to nothing, and the plan
# item it produced (P2-15) could not even be verified locally because no pinned
# version existed.
#
# ---------------------------------------------------------------------------
# IMPORTANT -- `format-check` is RED today, and that is not a bug in this file.
# ---------------------------------------------------------------------------
#
# Measured on `bcad264` (2026-11-24): 22 of the 23 files in modules/core deviate,
# 687 changed lines. The tree has never been formatted, so of course it does.
#
# It is also worth knowing before you run `make format` that clang-format's output
# here is NOT whitespace-only, which is what the plan assumed. Verified by
# comparing the token stream of every file before and after:
#
#   - `#define UNUSED(expr) do { (void)(expr); } while (0)` in filter_helpers.hpp
#     is split across four lines with backslash continuations.
#   - A string literal in the same header is split: "the only argument supplied"
#     becomes "the only argument" "supplied". Adjacent-literal concatenation makes
#     the runtime string identical, but it is a content change and it breaks
#     grepping for that message.
#   - Block comments get their `*` realigned, and `// namespace toffy` closing
#     comments are added where they were absent.
#
# All of it is semantically inert -- but "whitespace-only commit" is not an
# accurate description, so the reformat commit is a decision rather than a
# checkbox, and the CI gate lands in that same commit rather than before it.
# See cleanup/plan/build-and-hygiene.md (P2-15).

# The major version this tree is formatted with. Different clang-format majors
# disagree about output, so a gate that accepts any version is a gate that flips
# depending on which box runs it.
set(TOFFY_CLANG_FORMAT_MAJOR "18" CACHE STRING
    "clang-format major version this tree is formatted with")

# Prefer the pinned major by name, fall back to whatever is on PATH so the target
# still works on a box that has only `clang-format`.
find_program(TOFFY_CLANG_FORMAT_EXECUTABLE
    NAMES clang-format-${TOFFY_CLANG_FORMAT_MAJOR} clang-format
    DOC "clang-format used by the format / format-check targets")

if(NOT TOFFY_CLANG_FORMAT_EXECUTABLE)
  # A stub rather than silently omitting the targets: `make format` should say why
  # it cannot, not "No rule to make target".
  add_custom_target(format
      COMMAND ${CMAKE_COMMAND} -E echo
          "clang-format not found: install clang-format-${TOFFY_CLANG_FORMAT_MAJOR}"
      COMMAND ${CMAKE_COMMAND} -E false)
  add_custom_target(format-check
      COMMAND ${CMAKE_COMMAND} -E echo
          "clang-format not found: install clang-format-${TOFFY_CLANG_FORMAT_MAJOR}"
      COMMAND ${CMAKE_COMMAND} -E false)
  message(STATUS
      "clang-format ${TOFFY_CLANG_FORMAT_MAJOR} not found -- 'format' and "
      "'format-check' targets will report an error if used")
  return()
endif()

# Warn, rather than fail, on a version mismatch: a developer with clang-format 20
# can still read the diff, but they should know it will not agree with CI.
execute_process(
    COMMAND ${TOFFY_CLANG_FORMAT_EXECUTABLE} --version
    OUTPUT_VARIABLE toffy_cf_version
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
string(REGEX MATCH "version ([0-9]+)" toffy_cf_major_match "${toffy_cf_version}")
set(toffy_cf_major "${CMAKE_MATCH_1}")
if(NOT toffy_cf_major STREQUAL TOFFY_CLANG_FORMAT_MAJOR)
  message(WARNING
      "clang-format version mismatch: found '${toffy_cf_version}', this tree is "
      "formatted with major ${TOFFY_CLANG_FORMAT_MAJOR}. 'format-check' may report "
      "differences that the pinned version would not.")
endif()

# CONFIGURE_DEPENDS so files added to modules/core are picked up by a re-run of an
# existing build directory instead of silently escaping the gate.
file(GLOB_RECURSE TOFFY_FORMAT_FILES
     CONFIGURE_DEPENDS
     "${PROJECT_SOURCE_DIR}/modules/core/*.cpp"
     "${PROJECT_SOURCE_DIR}/modules/core/*.hpp")

list(LENGTH TOFFY_FORMAT_FILES toffy_format_count)

# Run from the source root so clang-format finds the repo-root .clang-format for
# every file regardless of how the target was invoked.
add_custom_target(format
    COMMAND ${TOFFY_CLANG_FORMAT_EXECUTABLE} -i ${TOFFY_FORMAT_FILES}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT
        "clang-format -i over ${toffy_format_count} files in modules/core "
        "(${toffy_cf_version})"
    VERBATIM)

add_custom_target(format-check
    COMMAND ${TOFFY_CLANG_FORMAT_EXECUTABLE} --dry-run -Werror ${TOFFY_FORMAT_FILES}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT
        "checking ${toffy_format_count} files in modules/core against .clang-format "
        "(${toffy_cf_version})"
    VERBATIM)

message(STATUS
    "formatting: clang-format ${toffy_cf_version} over ${toffy_format_count} "
    "files in modules/core -- targets 'format' and 'format-check'")
