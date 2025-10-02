# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.8)

# Project's name
project(liboculus)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -Wl,--no-as-needed")

# Set the output folder where your program will be created
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)

# ###########################################
# The following folders will be included  #
# ###########################################
include_directories(
    "${PROJECT_SOURCE_DIR}/include/"
    "${PROJECT_SOURCE_DIR}/thirdparty/"
)

# Threading
find_package(Threads)

# Boost
find_package(Boost 1.57 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIR})
message("Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIR})

# Install g3log as an external package
include(cmake/IncludeProject.cmake)

if(NOT GIT_TAG)
    set(GIT_TAG "2.6")
endif()

include(ExternalProject)
ExternalProject_Add(
    g3log
    GIT_REPOSITORY https://github.com/KjellKod/g3log
    GIT_TAG ${GIT_TAG}
    INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}/install/g3log"
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
)

ExternalProject_Get_Property(g3log install_dir)
include_directories(${install_dir}/include/)

# #####################
# Add Execuatables  #
# #####################
link_directories(${Boost_LIBRARY_DIRS})

# Create Library
add_library(oculus SHARED ${oculus_SRCS})
set_target_properties(oculus PROPERTIES LIBRARY_OUTPUT_NAME oculus)

# =============================================
# to allow find_package()
# =============================================
#
# The following is borrowed heavily from:
#   https://github.com/RossHartley/invariant-ekf
# I am responsible for all mistakes
#
# the following case be used in an external project requiring oculus:
# ...
# find_package(oculus)
# include_directories(${oculus_INCLUDE_DIRS})
# ...

# NOTE: the following will support find_package for 1) local build (make) and 2) for installed files (make install)

# 1- local build

# Register the local build in case one doesn't use "make install"
export(PACKAGE oculus)

# Create variable for the local build tree
# set_target_properties(oculus PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
get_property(
    oculus_include_dirs
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    PROPERTY INCLUDE_DIRECTORIES
)
get_property(
    oculus_library_dirs
    TARGET oculus
    PROPERTY LIBRARY_OUTPUT_DIRECTORY
)
get_property(oculus_libraries TARGET oculus PROPERTY LIBRARY_OUTPUT_NAME)

message("oculus_include_dirs: " ${oculus_include_dirs})
message("oculus_library_dirs: " ${oculus_library_dirs})
message("oculus_libraries: " ${oculus_libraries})

# Configure config file for local build tree
configure_file(
    cmake/oculusConfig.cmake.in
    "${PROJECT_BINARY_DIR}/oculusConfig.cmake"
    @ONLY
)

message("PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR})

# # 2- installation build #

# Change the include location for the case of an install location
set(oculus_include_dirs ${CMAKE_INSTALL_PREFIX}/include ${EIGEN_INCLUDE_DIR})

# We put the generated file for installation in a different repository (i.e., ./CMakeFiles/)
configure_file(
    cmake/oculusConfig.cmake.in
    "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/oculusConfig.cmake"
    @ONLY
)

install(
    FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/oculusConfig.cmake"
    DESTINATION share/oculus/cmake
    COMPONENT dev
)
