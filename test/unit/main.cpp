#include <gtest/gtest.h>
#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>

// Run all the declared gtests
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  
  auto worker = g3::LogWorker::createLogWorker();
  worker->addDefaultLogger(argv[0], ".");
  g3::initializeLogging(worker.get());

  return RUN_ALL_TESTS();
}
