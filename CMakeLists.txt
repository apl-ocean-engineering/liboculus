cmake_minimum_required(VERSION 3.8)
project(liboculus)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++17)

# == Code common to all builds =======================================
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(oculus_SRCS
    lib/DataRx.cpp
    lib/SonarConfiguration.cpp
    lib/SonarStatus.cpp
    lib/StatusRx.cpp
    lib/SonarPlayer.cpp
    lib/OculusMessageHandler.cpp
    lib/SimpleFireMessage.cpp
    lib/IoServiceThread.cpp
)

# ROS build, determine which version...
# QUIET will squelch the warning if the package isn't found
find_package(ros_environment QUIET)

if(${ros_environment_FOUND})
    set(ROS_VERSION $ENV{ROS_VERSION})

    if(${ROS_VERSION} EQUAL 2)
        # == ament/ROS2 section =================================

        find_package(ament_cmake REQUIRED)
        find_package(Boost REQUIRED COMPONENTS system)
        find_package(g3log_ros REQUIRED)

        add_library(oculus SHARED ${oculus_SRCS})
        target_link_libraries(oculus PUBLIC Boost::system g3log_ros::g3log)

        target_include_directories(
            oculus
            PUBLIC
                "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty>"
                "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
                "$<INSTALL_INTERFACE:include/thirdparty>"
        )

        install(
            TARGETS oculus
            EXPORT export_${PROJECT_NAME}
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
        )

        install(
            DIRECTORY include/${PROJECT_NAME} thirdparty
            DESTINATION include
            FILES_MATCHING
            PATTERN "*.hpp"
            PATTERN "*.h"
            PATTERN ".git" EXCLUDE
        )

        ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
        ament_export_libraries(oculus)

        ament_package()
    elseif(${ROS_VERSION} EQUAL 1)
        # Catkin/ROS1 section =====
        find_package(catkin REQUIRED COMPONENTS g3log_ros)
        find_package(Boost REQUIRED COMPONENTS system)

        catkin_package(
            CATKIN_DEPENDS g3log_ros
            INCLUDE_DIRS include thirdparty
            LIBRARIES liboculus
        )

        add_library(liboculus ${oculus_SRCS})

        include_directories(liboculus include thirdparty ${catkin_INCLUDE_DIRS})

        target_link_libraries(liboculus ${catkin_LIBRARIES})

        install(
            TARGETS liboculus
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

        ## Install headers
        install(
            DIRECTORY include/${PROJECT_NAME}/ thirdparty/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
            FILES_MATCHING
            PATTERN "*.hpp"
            PATTERN "*.h"
            PATTERN ".git" EXCLUDE
        )

        if(CATKIN_ENABLE_TESTING)
            add_definitions(
                -DTEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
            )
            include_directories(test/data/)

            file(GLOB oculus_test_SRCS test/unit/*cpp)

            catkin_add_gtest(oculus_test ${oculus_test_SRCS})

            target_link_libraries(
                oculus_test
                ${catkin_LIBRARIES}
                liboculus
                Boost::system
            )
        endif()
    else()
        message(
            "Unsure what sort of build to do.  Not in a ROS1 or ROS2 environment"
        )
    endif()
else()
    message(NOTICE "!! Performing non-ROS build !!")
    include(cmake/BuildNonROS.cmake)
endif()
