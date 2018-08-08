cmake_minimum_required(VERSION 3.5)
if( NOT FIPS_IMPORT )
	cmake_minimum_required(VERSION 3.5)

	get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
	include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

	fips_setup(PROJECT liboculus)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( Boost REQUIRED COMPONENTS system )

## Find Blueprint Oculus SDK
find_package( OculusSDK REQUIRED )

## Set global include paths
fips_include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${OCULUS_SDK_ROOT}
 )

## Verbose output about ASIO handlers
##add_definitions( -DBOOST_ASIO_ENABLE_HANDLER_TRACKING )


fips_add_subdirectory("lib/")

if( NOT FIPS_IMPORT )
  add_subdirectory("tools/")

	if( FIPS_UNITTESTS )
			add_subdirectory( test/unit/ )
  endif()

  fips_finish()
endif()
