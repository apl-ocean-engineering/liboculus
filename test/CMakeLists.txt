
add_definitions(-DTEST_DATA_PATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
fips_include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/data )


add_subdirectory( unit/ )
