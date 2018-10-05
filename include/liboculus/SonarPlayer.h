#pragma once

#include <fstream>

#include "liboculus/SimplePingResult.h"

namespace liboculus {


  class SonarPlayer {
  public:
    SonarPlayer();

    bool open( const std::string &filename );

    std::shared_ptr<SimplePingResult> nextPing();

    char *nextPacket();

  private:

    std::ifstream _input;

  };

}
