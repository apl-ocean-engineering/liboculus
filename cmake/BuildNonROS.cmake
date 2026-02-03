# Specify the minimum version for CMake
cmake_minimum_required(VERSION 3.8)
project(liboculus)

if(MSVC)
    add_compile_options(
        $<$<CONFIG:Release>:/O2>
        $<$<CONFIG:RelWithDebInfo>:/O2>
        $<$<CONFIG:MinSizeRel>:/O1>
    )
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(
        $<$<CONFIG:Release>:-O3>
        $<$<CONFIG:RelWithDebInfo>:-O3>
        $<$<CONFIG:MinSizeRel>:-Os>
        $<$<NOT:$<CONFIG:Debug>>:-march=native>
    )
    if(UNIX AND NOT APPLE)
        add_link_options(-Wl,--no-as-needed)
    endif()
endif()

# Set the output folder where your program will be created
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)

# ###########################################
# The following folders will be included  #
# ###########################################
include_directories("${PROJECT_SOURCE_DIR}/include/")

# Dependencies
option(LIBOCULUS_FETCH_DEPS "Fetch fmt/spdlog if not found" OFF)
option(LIBOCULUS_FORCE_FETCH_DEPS "Always fetch fmt/spdlog (ignore system)" OFF)

if(LIBOCULUS_FETCH_DEPS)
    include(FetchContent)
endif()

# fmt
if(LIBOCULUS_FETCH_DEPS)
    if(LIBOCULUS_FORCE_FETCH_DEPS)
        set(fmt_FOUND OFF)
    else()
        find_package(fmt QUIET)
    endif()
    if(NOT fmt_FOUND)
        FetchContent_Declare(
            fmt
            GIT_REPOSITORY https://github.com/fmtlib/fmt.git
            GIT_TAG 10.2.1
        )
        FetchContent_MakeAvailable(fmt)
    endif()
else()
    find_package(fmt REQUIRED)
endif()

# spdlog
if(LIBOCULUS_FETCH_DEPS)
    if(LIBOCULUS_FORCE_FETCH_DEPS)
        set(spdlog_FOUND OFF)
    else()
        find_package(spdlog QUIET)
    endif()
    if(NOT spdlog_FOUND)
        set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "" FORCE)
        set(SPDLOG_FMT_EXTERNAL_HO OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(
            spdlog
            GIT_REPOSITORY https://github.com/gabime/spdlog.git
            GIT_TAG v1.13.0
        )
        FetchContent_MakeAvailable(spdlog)
    endif()
else()
    find_package(spdlog REQUIRED)
endif()

# Threading
find_package(Threads)

# Boost
find_package(Boost 1.57 REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIR})
message("Boost_INCLUDE_DIR: " ${Boost_INCLUDE_DIR})

include_directories(${install_dir}/include/)

# #####################
# Add Execuatables  #
# #####################
link_directories(${Boost_LIBRARY_DIRS})

# Create Library
if(MSVC)
    set(OCULUS_LIBRARY_TYPE STATIC)
else()
    set(OCULUS_LIBRARY_TYPE SHARED)
endif()

add_library(oculus ${OCULUS_LIBRARY_TYPE} ${oculus_SRCS})
set_target_properties(
    oculus
    PROPERTIES
        LIBRARY_OUTPUT_NAME oculus
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib/$<CONFIG>"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib/$<CONFIG>"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin/$<CONFIG>"
)
target_link_libraries(oculus PUBLIC fmt::fmt spdlog::spdlog)
if(LIBOCULUS_FETCH_DEPS AND DEFINED fmt_SOURCE_DIR)
    # Ensure fetched fmt headers are preferred over any system/Anaconda fmt
    target_include_directories(oculus BEFORE PUBLIC "${fmt_SOURCE_DIR}/include")
endif()
if(MSVC)
    target_compile_options(oculus PRIVATE /wd5208 /wd4996)
    target_compile_definitions(
        oculus PRIVATE _SILENCE_STDEXT_ARR_ITERS_DEPRECATION_WARNING
        _USE_MATH_DEFINES
        _WIN32_WINNT=0x0601
    )
    target_link_libraries(oculus PUBLIC Ws2_32)
endif()

add_executable(occlient ${PROJECT_SOURCE_DIR}/tools/oculus_client.cpp)
target_link_libraries(occlient oculus)
if(LIBOCULUS_FETCH_DEPS AND DEFINED fmt_SOURCE_DIR)
    target_include_directories(occlient BEFORE PUBLIC "${fmt_SOURCE_DIR}/include")
endif()
set_target_properties(
    occlient
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin/$<CONFIG>"
)

# =============================================
# to allow find_package()
# =============================================
#
# The following is borrowed heavily from:
# https://github.com/RossHartley/invariant-ekf
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

# 2- installation build #

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
