cmake_minimum_required(VERSION 3.10)

message (STATUS "CMake version: " ${CMAKE_VERSION} )

project( toffy )

SET(CMAKE_COLOR_MAKEFILE ON)
SET(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################################################################
# Cmake modules
################################################################################

#Including local extra cmake modules
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

#Function for generation of export macros for libraries
include(GenerateExportHeader) #(toffy_export.h - TOFFY_EXPORT)
# Include module with fuction 'write_basic_package_version_file' for ToffyConfig.cmake
include(CMakePackageConfigHelpers)


################################################################################
#Version stuff
################################################################################

# Make a version file containing the current version from git.
include(GetGitRevisionDescription)
git_describe(VERSION --tag --long)

##parse the version information into pieces.
string(REGEX REPLACE "^v([0-9]+)\\..*" "\\1" PROJECT_VERSION_MAJOR "${VERSION}")
string(REGEX REPLACE "^v[0-9]+\\.([0-9]+).*" "\\1" PROJECT_VERSION_MINOR "${VERSION}")
string(REGEX REPLACE "^v[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" PROJECT_VERSION_PATCH "${VERSION}")
string(REGEX REPLACE "^v[0-9]+\\.[0-9]+\\.[0-9]+(.*)" "\\1" PROJECT_VERSION_TWEAK "${VERSION}")
set(PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
set(PROJECT_DESCRIPTION "Toffy 3D computer vision framework")

set(PROJECT_VERSION_FULL "${PROJECT_VERSION}${PROJECT_VERSION_TWEAK}")

message (STATUS "version: " ${VERSION} )
message (STATUS ${PROJECT_NAME}" version: " ${PROJECT_VERSION_FULL} )

################################################################################
# User project configuration
################################################################################
option (BUILD_STATIC "Build static version of the library" OFF)
option (WITHOUT_PCL "Exclude pointclouds-based classes" OFF)
option (WITH_PCL_CLOUDVIEW "Build pcl-based viewer" OFF)
option (WITH_VISUALIZATION "Add viewer objects. Requires GUI" ON)
option (BUILD_TESTS "Build test code." ON)

message (STATUS "${CMAKE_PROJECT_NAME} options: ")
message (STATUS "\t BUILD_STATIC: " ${BUILD_STATIC})
message (STATUS "\t WITHOUT_PCL: " ${WITHOUT_PCL})
message (STATUS "\t WITH_VISUALIZATION: " ${WITH_VISUALIZATION})

################################################################################
# Install options
################################################################################
MESSAGE(STATUS "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})

set(INSTALL_LIB_DIR lib/ CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin/ CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include/${PROJECT_NAME} CACHE PATH "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR cmake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")



#Collect project definitions
set(DEFINITIONS)
set(LIBS)

#
# devel settings:
#
set(DEVEL_USER $ENV{USER})
set(USER_${DEVEL_USER} 1)

################################################################################
# Build types stuff
################################################################################
set( CMAKE_CONFIGURATION_TYPES "Debug;Release" )

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
   set(CMAKE_BUILD_TYPE Release CACHE STRING "build type default to Release" FORCE)
endif("${CMAKE_BUILD_TYPE}" STREQUAL "")

if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(${PROJECT_NAME}_DEBUG 1)
    add_definitions(-DCM_DEBUG )
endif()
MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_DEBUG_POSTFIX -d)

if (${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(PLAT_WINDOWS "yes")
    add_definitions(-DPLAT_WINDOWS)
endif()
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(PLAT_LINUX "yes")
    add_definitions(-DPLAT_LINUX)
endif()
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(PLAT_APPLE "yes")
    add_definitions(-DPLAT_APPLE)

    # for macports:
    include_directories ("/opt/local/include")
    link_directories("/opt/local/lib")
endif()

################################################################################
# Install options
################################################################################
MESSAGE(STATUS "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX})

set(INSTALL_LIB_DIR lib/ CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin/ CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include/${PROJECT_NAME} CACHE PATH "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR cmake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")


#
# set these early to override pollution by installed versions
#
include_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/libraries/include/"
        "${CMAKE_CURRENT_SOURCE_DIR}/modules/core/include/"
        "${CMAKE_CURRENT_SOURCE_DIR}/modules/bta/include/"
        "${CMAKE_CURRENT_SOURCE_DIR}/modules/filters/include/"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/"        
        )

################################################################################
# Build config windows - unix - arm
################################################################################
message (STATUS "MSVC: " ${MSVC})
if(MSVC)
    #Generated ProductVersionFiles --> https://github.com/halex2005/CMakeHelpers
    #VersionInfo.h & VersionResource.h
    include(generate_product_version)
    generate_product_version(
       ProductVersionFiles
       NAME ${PROJECT_NAME}
       ICON ${PATH_TO_APPLICATION_ICON}
       VERSION_MAJOR ${PROJECT_VERSION_MAJOR}
       VERSION_MINOR ${PROJECT_VERSION_MINOR}
       VERSION_PATCH ${PROJECT_VERSION_PATCH}
       VERSION_REVISION ${BUILD_REVISION}
       COMPANY_NAME VoXel Interaction Design
       FILE_DESCRIPTION ${toffy_DESCRIPTION}
    )

    #Set MSVC to preprocessor
    list(APPEND DEFINITIONS -DMSVC)

    #Set windows preprocesor definitions
    list(APPEND DEFINITIONS -D_WIN32_WINNT=0x0601) #Win10 0501-> Win7

    #Nedeed?
    # By default, do not warn when built on machines using only VS Express:
    #IF(NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
    #  SET(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
    #ENDIF()

    # Force to always compile with W4
    #if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    #    string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    #else()
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    #endif()

    #Build static version of the library for windows
    if(BUILD_STATIC)
        #We link statically to reduce dependancies
        foreach(flag_var CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
            if(${flag_var} MATCHES "/MD")
                string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
            endif(${flag_var} MATCHES "/MD")
            if(${flag_var} MATCHES "/MDd")
                string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
            endif(${flag_var} MATCHES "/MDd")
        endforeach(flag_var)
    endif()

    #Set where is the BtaTofApi in windows, In unix we asume BtaConfig.cmake installed
    #set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/Findbta.cmake)
else(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#Unix build options

    # TODO check compile options for build type, Debug, Release, ....
    list(APPEND DEFINITIONS -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT -DEIGEN_SUPPORT_STAGE=99)
    add_compile_options(-pthread -g -ggdb -Wall -Wextra -Wno-long-long)
    #ARM complains if pthread not added explicitly
    list(APPEND LIBS pthread)
endif()

#######################################
# Build static of dynamic version of the lib. Common options
#######################################
if(BUILD_STATIC)
#Ajust to include also dependencies as static
    set(OpenCV_STATIC ON)
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_STATIC_RUNTIME ON)
    set (LIB_LINK STATIC)
else(NOT BUILD_STATIC)
#Shared obtions
    list(APPEND DEFINITIONS -DBOOST_ALL_DYN_LINK -DBOOST_ALL_NO_LIB) #Pcl log and ???, dynamic
    set (LIB_LINK SHARED)
endif()
        
################################################################################
# Find dependencies
################################################################################

# rapidjson
#include(ExternalProject)
#include("${CMAKE_SOURCE_DIR}/cmake/rapidjson.cmake")
#include_directories(
#    ${RAPIDJSON_INCLUDE_DIR}
#)
#    ${CMAKE_SOURCE_DIR}/include
#    ${GFLAGS_INCLUDE_DIR}
#    ${BENCHMARK_INCLUDE_DIR}


# Boost
find_package(Boost
    COMPONENTS log log_setup program_options system
        filesystem date_time serialization chrono thread
    REQUIRED)
#include_directories(${Boost_INCLUDE_DIRS})
list(APPEND LIBS ${Boost_LIBRARIES})
add_definitions(-DBOOST_ALL_DYN_LINK )

if(NOT MSVC)
    find_package(PkgConfig)
    pkg_search_module(Eigen3 REQUIRED eigen3)
    include_directories(${Eigen3_INCLUDE_DIRS})
endif()

# PCL
if(NOT WITHOUT_PCL)
    find_package(PCL COMPONENTS common filters io sample_consensus segmentation features visualization)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    list(APPEND LIBS ${PCL_LIBRARIES})
    message (STATUS "PCL_INCLUDE_DIRS: " ${PCL_INCLUDE_DIRS})
    message (STATUS "PCL_LIBRARIES: " ${PCL_LIBRARIES})
    message (STATUS "PCL_VIZ: " ${PCL_VISUALIZATION_FOUND})

    add_definitions(-DPCL_FOUND=1)
    message (STATUS "PCL_FOUND: " ${PCL_FOUND})
    if (NOT PCL_FOUND)
        set(WITHOUT_PCL 1)
    endif()
endif()

# OpenCV

find_package( OpenCV COMPONENTS core imgproc highgui calib3d videoio OPTIONAL_COMPONENTS viz tracking REQUIRED )
message("  OpenCV libs " ${OpenCV_LIBS} )
list(APPEND LIBS ${OpenCV_LIBS})
message("OpenCV " ${OpenCV_VERSION_MAJOR} "." ${OpenCV_VERSION_MAJOR} "." ${OpenCV_VERSION_PATCH})
add_definitions(-DOCV_VERSION_MAJOR=${OpenCV_VERSION_MAJOR} -DOCV_VERSION_MINOR=${OpenCV_VERSION_MINOR} -DOCV_VERSION_PATCH=${OpenCV_VERSION_PATCH})

if(TARGET opencv_viz)
    message("  OpenCV modules: VIZ FOUND " ${OpenCV_LIBS} )
endif()
if(TARGET opencv_tracking)
    message("  OpenCV modules: TRACKING FOUND " ${OpenCV_LIBS} )
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

list(APPEND LIBS ${OpenCV_LIBS})

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-O2 -fPIC)

#
# bta library -- communication with Becom Systems cameras (Argos, Sentis, ...)
#
find_package( bta )
if (${bta_FOUND})
    message(INFO " bta inc  ${bta_INCLUDE_DIRS}")
    message(INFO " bta lib  ${bta_LIBRARIES}")
    message(INFO " bta defs ${bta_DEFINITIONS}")
    include_directories(${bta_INCLUDE_DIRS})
    list(APPEND LIBS ${bta_LIBRARIES} )
    set(HAS_BTA 1)
    add_definitions(-DHAS_BTA=1)

    if (PLAT_LINUX)
        # for shm_*
        list(APPEND LIBS -lrt)
    endif()
else()
    message(WARNING "no bta library!")
endif()

add_definitions(-Wall  )
# add_definitions(-Weffc++ )

add_subdirectory(libraries)

add_subdirectory(modules)


# ################################################################################
# # Create lib and target
# ################################################################################

add_library( ${PROJECT_NAME} SHARED 
    # libs
    $<TARGET_OBJECTS:toffy_libs> 
    $<TARGET_OBJECTS:toffy_lib_common> 
    $<TARGET_OBJECTS:toffy_lib_tracer> 
    $<TARGET_OBJECTS:toffy_lib_graphs> 
    $<TARGET_OBJECTS:toffy_lib_skel> 
    $<TARGET_OBJECTS:toffy_tracking> 

    # modules:
    $<TARGET_OBJECTS:toffy_bta> 
    $<TARGET_OBJECTS:toffy_core> 
    # filters
    $<TARGET_OBJECTS:toffy_filters> 

    $<TARGET_OBJECTS:toffy_base> 
    $<TARGET_OBJECTS:toffy_3d> 
    $<TARGET_OBJECTS:toffy_capture> 
    $<TARGET_OBJECTS:toffy_detection> 
    $<TARGET_OBJECTS:toffy_import> 
    $<TARGET_OBJECTS:toffy_reproject> 
    $<TARGET_OBJECTS:toffy_smoothing> 
    $<TARGET_OBJECTS:toffy_tracking> 
    $<TARGET_OBJECTS:toffy_viewers> 
    )

target_link_libraries( ${PROJECT_NAME} PUBLIC    ${LIBS}  )
set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${PROJECT_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${PROJECT_VERSION})
install (TARGETS ${PROJECT_NAME} 
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)


# #Build target dir for config, export and package files
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# # Add definitions for targets
# # Values:
# #   * Debug: -Dtoffy_DEBUG=1
# #   * Release: -Dtoffy_DEBUG=0
# #   * other: -Dtoffy_DEBUG=X
# #message (DEBUG "DEFINITIONS: ${DEFINITIONS}")
# target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEFINITIONS})
# # toffy_DEBUG only get set alone
# target_compile_definitions(${PROJECT_NAME} PUBLIC "${PROJECT_NAME}_DEBUG=$<CONFIG:Debug>")


# # Generate Export header toffy_export.h with toffy_EXPORT:
set(export_h "${generated_dir}/${PROJECT_NAME}/${PROJECT_NAME}_export.h")
GENERATE_EXPORT_HEADER(${PROJECT_NAME}
    EXPORT_FILE_NAME ${export_h}  
)
list(APPEND headers ${export_h})
install(
   FILES ${export_h}
   DESTINATION "${INSTALL_INCLUDE_DIR}/"
)


# Generate Config file toffy_config.h
set(config_h_in "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}_config.h.in")
set(config_h "${generated_dir}/${PROJECT_NAME}/${PROJECT_NAME}_config.h")
configure_file("${config_h_in}" "${config_h}")
list(APPEND headers ${config_h})
install(
   FILES ${config_h}
   DESTINATION "${INSTALL_INCLUDE_DIR}/common"
)

#######################################
# Tests
#######################################

if( ${BUILD_TESTS})
    add_subdirectory(tests)
endif()

#######################################
# Demo app
#######################################

add_subdirectory(apps)

################################################################################
# Install Stuff
################################################################################

#######################################
# Config cmake
#######################################
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake") #output
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake") #output

#Targets name
#   * ${INSTALL_CMAKE_DIR}/${targets_export_name}.cmake
set(targets_export_name "${PROJECT_NAME}Targets")
set(namespace "${PROJECT_NAME}::")

# Set version required to config
write_basic_package_version_file(
    "${version_config}"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>Config.cmake'
configure_package_config_file("cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}"
)

# Instal config
#   * <prefix>/lib/cmake/toffy/toffyConfig.cmake --> ${project_config}
#   * <prefix>/lib/cmake/toffy/toffyConfigVersion.cmake --> ${version_config}
install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${INSTALL_CMAKE_DIR}"
)

install(
    EXPORT "${targets_export_name}"
    NAMESPACE "${namespace}"
    DESTINATION "${INSTALL_CMAKE_DIR}/"
)

#Install
#if( NOT BUILD_STATIC)
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${targets_export_name} 
    LIBRARY
        DESTINATION "${INSTALL_LIB_DIR}" COMPONENT library INCLUDES NAMELINK_SKIP
    ARCHIVE
        DESTINATION lib
    RUNTIME
        DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    INCLUDES
        DESTINATION "${INSTALL_INCLUDE_DIR}"
    #We create only the include folder. Our headers have a specfic folder hierarchy
    PUBLIC_HEADER
        DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT headers
	)

INSTALL( PROGRAMS ${binaries}
    DESTINATION ${INSTALL_BIN_DIR}
)


#else()
#install(TARGETS toffy
#    EXPORT ${targets_export_name}
#    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT library  INCLUDES NAMELINK_SKIP
#    ARCHIVE DESTINATION lib
#    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
#    PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT library
#)
#endif()

# #######################################
# # Install well looking header
# #######################################

# #Look for all headers and get path fomarted nicely
# foreach( file ${headers} )
#     #message (DEBUG "dir: ${file}")
#     get_filename_component( dir ${file} DIRECTORY )
#     #message (DEBUG "dir: ${dir}")
#     if(NOT dir MATCHES "(.*)toffy/")
#         LIST( APPEND headers_dirs "${dir}" )
#     else()
#         string(FIND "${dir}" "toffy" pos REVERSE)
#         MATH(EXPR pos "${pos}+4")
#         #MATH(EXPR <output variable> <math expression>)
#         string(SUBSTRING "${dir}" 0 "${pos}" subdir)
#         #message (DEBUG "dir: ${dir}")
#         #message (DEBUG "pos: ${pos}")
#         #message (DEBUG "subdir: ${subdir}")
#         LIST( APPEND headers_dirs "${subdir}" )
#     endif()
# endforeach()
# list( REMOVE_DUPLICATES headers_dirs )

# #Install toffy headers
# INSTALL( FILES ${headers}
#     #DIRECTORY ${headers_dirs}
#     #DESTINATION include
#     DESTINATION ${INSTALL_INCLUDE_DIR}
#     COMPONENT headers
#     #FILES_MATCHING PATTERN "*.hpp"
# )

#######################################
# Copying headers to build directory for windows
#######################################
if(MSVC)
    FILE( COPY ${headers_dirs} DESTINATION include/ FILES_MATCHING PATTERN "*.hpp")
    FILE( COPY ${config_h} DESTINATION include/${PROJECT_NAME}/common/ FILES_MATCHING PATTERN "*.h")
    FILE( COPY ${export_h} DESTINATION include/${PROJECT_NAME}/common/ FILES_MATCHING PATTERN "*.h")
    export(TARGETS ${PROJECT_NAME} FILE "${PROJECT_BINARY_DIR}/generated/${PROJECT_NAME}Targets.cmake")
endif()

#######################################
# uninstall target
#######################################
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)

#######################################
# Generating .pc file for pk-config
#######################################
# SET(PKG_CONFIG_REQUIRES "opencv boost") # TODO bta missing pk, pcl?
SET(PKG_CONFIG_LIBDIR
    "\${prefix}/lib/"
)
SET(PKG_CONFIG_INCLUDEDIR
    "\${prefix}/include/"
)
SET(PKG_CONFIG_LIBS
    "-L\${libdir} -l${PROJECT_NAME}"
)
SET(PKG_CONFIG_CFLAGS
    "-I\${includedir}"
)

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pkg-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc"
)

install(
    FILES "${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc"
    DESTINATION lib/pkgconfig
)

################################################################################
# Documentation
################################################################################
# add a target to generate API documentation with Doxygen
include(docs/CMakeLists.txt OPTIONAL)

################################################################################
# Test
################################################################################
# TODO move to a file
#CPACK
include(InstallRequiredSystemLibraries)

SET (CPACK_GENERATOR "DEB;TGZ")
SET (CPACK_SOURCE_GENERATOR "TGZ")
SET (CPACK_INCLUDE_TOPLEVEL_DIRECTORY "1")
SET (CPACK_PACKAGE_NAME ${PROJECT_NAME})
set (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set (CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
set (CPACK_RESOURCE_FILE_WELCOME "${CMAKE_CURRENT_SOURCE_DIR}/Welcome.txt")
#set (CPACK_PACKAGE_ICON ".ico")
set (CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
SET (CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CMAKE_SYSTEM_NAME}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CMAKE_SYSTEM_NAME}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

SET (CPACK_PACKAGE_VENDOR " VoXel Interaction Design ")
SET (CPACK_PACKAGE_DESCRIPTION "Linz 3D computer vision library")
#set (CPACK_PACKAGE_DESCRIPTION_FILE "description.txt")

SET (CPACK_PACKAGE_DESCRIPTION_SUMMARY "Linz 3D computer vision library")
SET (CPACK_PACKAGE_CONTACT "VoXel Interaction Design <office@voxel.at>")
SET (CPACK_SOURCE_IGNORE_FILES ".*~;")

if ("${CPACK_GENERATOR}" STREQUAL "DEB")
    set(CPACK_DEB_COMPONENT_INSTALL ON)
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
    if(LINUX32)
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
    endif(LINUX32)
    set(CPACK_DEBIAN_PACKAGE_NAME ${CPACK_PACKAGE_NAME})
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER ${CPACK_PACKAGE_CONTACT})
    set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "http://www.voxel.at/")
endif()
if ("${CPACK_GENERATOR}" STREQUAL "TGZ")
    set(CPACK_TGZ_COMPONENT_INSTALL ON)
    set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
    set(CPACK_COMPONENT_INCLUDE_TOPLEVEL_DIRECTORY ON)
    set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)
endif()
SET (CPACK_TOPLEVEL_TAG ".${CMAKE_SYSTEM_NAME}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
SET (CPACK_OUTPUT_FILE_PREFIX "${CMAKE_SYSTEM_NAME}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
include (CPack)
