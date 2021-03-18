fips_begin_app( oc_client cmdline )
  fips_files(
    oculus_client.cpp
  )

  fips_deps( oculus )

  fips_include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
  )

fips_end_app()
