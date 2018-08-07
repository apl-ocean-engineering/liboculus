# Set source files

fips_begin_module( oculus )
  fips_src( . )

  ## The Blackmagic API is distributed as source and will be compiled into
  ## this library
  # fips_files(
  #   ${BLACKMAGIC_SRCS}
  # )

  fips_deps( logger )
  fips_libs( Boost::system )
  #
  # fips_include_directories(
  #   ${CMAKE_CURRENT_SOURCE_DIR}
  #   ${CMAKE_SOURCE_DIR}/include/libblackmagic
  # )
fips_end_module()
