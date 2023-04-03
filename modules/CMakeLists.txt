add_subdirectory(core)

add_subdirectory(bta)

add_subdirectory(filters)

add_subdirectory(control)

# add_library(toffy_modules OBJECT
#     $<TARGET_OBJECTS:toffy_core> 
#     $<TARGET_OBJECTS:toffy_bta> 
#     $<TARGET_OBJECTS:toffy_filters> 
#     init.cpp 
#     )

# target_link_libraries(toffy_modules 
#     )
# install (TARGETS toffy_modules
#          ARCHIVE DESTINATION lib
#          LIBRARY DESTINATION lib
#          RUNTIME DESTINATION bin)
