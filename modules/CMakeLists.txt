add_subdirectory(core)

# The bta module was added unconditionally, so it compiled even when
# find_package(bta) failed -- and linking then died with ~162 undefined BTA*
# references. filterfactory.cpp already guards its include and its use of
# toffy::bta::init() with #ifdef HAS_BTA, so only the build wiring was wrong.
# The single consumer of $<TARGET_OBJECTS:toffy_bta> is gated the same way.
if (HAS_BTA)
    add_subdirectory(bta)
endif()

add_subdirectory(filters)

