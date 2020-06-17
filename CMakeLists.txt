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

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( Boost REQUIRED COMPONENTS system )

# Actually not using full SDK, just the header files
# in thirdparty/Oculus
#find_package( OculusSDK REQUIRED )

if (FIPS_CONFIG)
	#== fips-specific section ==========================================

	add_definitions( -DWITH_GPMF )

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

		if( FIPS_UNITTESTS )
			add_subdirectory( test/ )
		endif()

		fips_finish()
	endif()

else()
	# == ROS / Catkin-specific section =================================

	project(liboculus)

	find_package(catkin REQUIRED
		#cmake_modules
		active_object REQUIRED
		g3log_catkin
	)

	catkin_package(
		CATKIN_DEPENDS active_object g3log_catkin
		INCLUDE_DIRS include thirdparty
		LIBRARIES liboculus_LIB
	)


	## C++ flags applied to all builds
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

	include_directories(
		include
		thirdparty
		${catkin_INCLUDE_DIRS}
	)

	file(GLOB oculus_SRCS
		lib/*cpp
		#thirdparty/gpmf-parser/*.c
	)


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
			test/unit/*cpp
		)
	  catkin_add_gtest(oculus_test ${oculus_test_SRCS})

	  target_link_libraries(oculus_test ${catkin_LIBRARIES}
							liboculus_LIB
							Boost::system )
	endif()


endif()
