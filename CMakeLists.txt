cmake_minimum_required(VERSION 3.5)
project(liboculus)

if( FIPS_CONFIG AND NOT FIPS_IMPORT )
  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

  fips_setup()
else()
  find_package(catkin QUIET)
endif()

# == Code common to all builds =======================================

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package( Boost REQUIRED COMPONENTS system )

# Actually not using full SDK, just the header files
# in thirdparty/Oculus
#find_package( OculusSDK REQUIRED )

if (FIPS_CONFIG)
  #== fips-specific section ==========================================
  add_compile_options(-std=c++11)

  ## Set global include paths
  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
  )

  ## Verbose output about ASIO handlers
  #add_definitions( -DBOOST_ASIO_ENABLE_HANDLER_TRACKING )

  fips_add_subdirectory("lib/")

  if( NOT FIPS_IMPORT )
    add_subdirectory("tools/")
  endif()

  if( FIPS_UNITTESTS )
    add_subdirectory( test/ )
  endif()

  fips_finish()

else()
  # == ROS / Catkin-specific section =================================
  project(liboculus)

  find_package(catkin REQUIRED
          #cmake_modules
          g3log_ros )

  catkin_package(
            CATKIN_DEPENDS g3log_ros
            INCLUDE_DIRS include thirdparty
            LIBRARIES liboculus_LIB )

  add_compile_options(-std=c++14)

  include_directories(
              include
              thirdparty
              ${catkin_INCLUDE_DIRS} )

  set(oculus_SRCS
              lib/SimplePingResult.cpp
              lib/DataRx.cpp
              lib/SonarConfiguration.cpp
              lib/SonarStatus.cpp
              lib/StatusRx.cpp
              lib/SonarPlayer.cpp )


  add_library(liboculus_LIB ${oculus_SRCS})
  target_link_libraries(liboculus_LIB ${catkin_LIBRARIES})

  install(TARGETS liboculus_LIB
              ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
              LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
              RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

  ## Install headers
  install(DIRECTORY  include/${PROJECT_NAME}/ thirdparty/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
              PATTERN ".git" EXCLUDE)

  if (CATKIN_ENABLE_TESTING)
    add_definitions(-DTEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    include_directories( test/data/ )

    file(GLOB oculus_test_SRCS
              test/unit/*cpp )

    catkin_add_gtest(oculus_test ${oculus_test_SRCS})

    target_link_libraries(oculus_test ${catkin_LIBRARIES}
              liboculus_LIB
              Boost::system )
  endif()


endif()
