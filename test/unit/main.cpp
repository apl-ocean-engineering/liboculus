
#include <gtest/gtest.h>
#include <libg3logger/g3logger.h>

// Run all the declared gtests
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  libg3logger::G3Logger logWorker( argv[0] );
  //logWorker.setLevel( DEBUG );

  return RUN_ALL_TESTS();
}
