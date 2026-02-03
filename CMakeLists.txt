cmake_minimum_required(VERSION 3.14.4)
project(liboculus)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT MSVC)
    add_compile_options(-std=c++17)
endif()

# == Code common to all builds =======================================
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(oculus_SRCS
    src/DataRx.cpp
    src/SonarConfiguration.cpp
    src/SonarStatus.cpp
    src/StatusRx.cpp
    src/SonarPlayer.cpp
    src/OculusMessageHandler.cpp
    src/SimpleFireMessage.cpp
    src/IoServiceThread.cpp
)

# ROS build, determine which version...
if(DEFINED ENV{ROS_DISTRO})
    find_package(ros_environment)

    set(ROS_VERSION $ENV{ROS_VERSION})

    if(${ROS_VERSION} EQUAL 2)
        message(NOTICE "!! Performing ROS2 build !!")
        include(cmake/BuildROS2.cmake)
    elseif(${ROS_VERSION} EQUAL 1)
        message(NOTICE "!! Performing ROS1 build !!")
        include(cmake/BuildROS1.cmake)
    else()
        message(
            "Unsure what sort of build to do.  Not in a ROS1 or ROS2 environment"
        )
    endif()
else()
    message(NOTICE "!! Performing non-ROS build !!")
    include(cmake/BuildNonROS.cmake)
endif()
