
#include <gtest/gtest.h>

// Run all the declared gtests
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  //auto worker = g3::LogWorker::createLogWorker();
  //libg3logger::G3Logger logWorker( argv[0] );
  //logWorker.setLevel( DEBUG );

  return RUN_ALL_TESTS();
}
