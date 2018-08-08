

#include <libg3logger/g3logger.h>
#include <CLI/CLI.hpp>


#include "liboculus/StatusRx.h"


using namespace liboculus;

int main( int argc, char **argv ) {

  libg3log::G3Logger logger("bmRecorder");

  CLI::App app{"Simple Oculus Sonar app"};

  try {
    boost::asio::io_service io_service;

    OsStatusRx statusRx( io_service );

    io_service.run();
  }
  catch (std::exception& e)
  {
    LOG(WARNING) << "Exception: " << e.what();
  }

}
