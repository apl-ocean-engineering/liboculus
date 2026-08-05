# Catkin/ROS1 section =====
find_package(catkin REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost OPTIONAL_COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES liboculus
)

add_library(liboculus ${oculus_SRCS})

include_directories(liboculus include ${catkin_INCLUDE_DIRS})

target_link_libraries(liboculus ${catkin_LIBRARIES})

install(
    TARGETS liboculus
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install headers
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
    PATTERN ".git" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
    add_definitions(-DTEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    include_directories(test/)

    file(GLOB oculus_test_SRCS test/unit/*cpp)

    catkin_add_gtest(oculus_test ${oculus_test_SRCS})

    target_link_libraries(
        oculus_test
        ${catkin_LIBRARIES}
        liboculus
        Boost::system
        spdlog::spdlog
    )
endif()
