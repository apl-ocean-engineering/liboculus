#pragma once

#include <fstream>

#include "liboculus/SonarPlayer.h"
#include "gpmf-parser/GPMF_parser.h"


namespace liboculus {

  ///
  ///
  ///
  class GPMFSonarPlayer : public SonarPlayerBase {
  public:
    GPMFSonarPlayer();
    virtual ~GPMFSonarPlayer();

    virtual bool open( const std::string &filename );
    virtual bool isOpen( ) const { return _valid; }

    void close();

    virtual bool eof();
    virtual void rewind();

    //char *nextPacket();
    virtual std::shared_ptr<SimplePingResult> nextPing();

    void dumpGPMF( void );

  private:

    GPMF_stream _stream;
    bool _valid;
    std::string _buffer;
  };

}
