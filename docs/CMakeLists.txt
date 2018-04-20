
# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
    SET(DOXYGEN_INPUT docs/Doxyfile.cfg)
    file(GLOB_RECURSE dox docs/*.dox docs/*.xml)
    SET(DOXYGEN_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    SET (DOCS_OUTPUT_FOLDER internal)
    if(CMAKE_BUILD_TYPE MATCHES Debug)
        #SET (ENABLE_PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR})
        SET (DOCS_OUTPUT_FOLDER public)
    endif()

    configure_file(${DOXYGEN_INPUT} ${DOXYGEN_OUTPUT} @ONLY)

    #add ALL for generating in make
    add_custom_target(docs DEPENDS ${DOXYGEN_OUTPUT}
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUTPUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        SOURCES ${dox}
        VERBATIM
    )

    #install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html DESTINATION share/doc)
endif(DOXYGEN_FOUND)

