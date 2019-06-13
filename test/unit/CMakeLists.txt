

## NO_TEMPLATE == we provide main()
##  Our main initializes g3log
gtest_begin( oculus NO_TEMPLATE )
    fips_src( . )
    fips_deps( oculus )
gtest_end()
