# Set source files

fips_begin_module( oculus )
  fips_src( . )

  fips_deps( g3logger )
  fips_libs( Boost::system )

fips_end_module()
