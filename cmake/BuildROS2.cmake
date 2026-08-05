# == ament/ROS2 section =================================

find_package(ament_cmake REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost OPTIONAL_COMPONENTS system)

add_library(oculus SHARED ${oculus_SRCS})
target_link_libraries(oculus PUBLIC
     Boost::system
     spdlog::spdlog)

target_include_directories(
    oculus
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/>"
)

install(
    TARGETS oculus
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
    FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
    PATTERN ".git" EXCLUDE
)

ament_export_dependencies(spdlog)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(oculus)

ament_package()
