#pragma once

#include <fstream>

#include "liboculus/SimplePingResult.h"

namespace liboculus {


  class SonarPlayer {
  public:
    SonarPlayer();

    bool open( const std::string &filename );
    bool isOpen( ) const { return _input.is_open(); }

    bool eof() const { return _input.eof(); }
    void rewind() { _input.seekg(0); }

    std::shared_ptr<SimplePingResult> nextPing();

    char *nextPacket();

  private:

    std::ifstream _input;

  };

}
