cmake_minimum_required(VERSION 3.5)

find_package(catkin QUIET)
if( NOT FIPS_IMPORT)
    if (NOT catkin_FOUND)
	cmake_minimum_required(VERSION 3.5)

	get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
	include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

	fips_setup(PROJECT liboculus)
    endif()
endif()



list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( Boost REQUIRED COMPONENTS system )

# Actually not using full SDK, just the header files
# in thirdparty/Oculus
#find_package( OculusSDK REQUIRED )

if (FIPS_IMPORT)
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
    include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
      )

    file(GLOB oculus_SRCS
	lib/*cpp
      )

    add_library(oculus_core ${oculus_SRCS})

    install(TARGETS oculus_core
            DESTINATION lib)   


    ## Install headers
    install(DIRECTORY  include/ lib/ thirdparty/Oculus/
        DESTINATION include
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
        PATTERN ".git" EXCLUDE)
endif()
