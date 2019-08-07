#pragma once

#include <fstream>

#include "liboculus/SimplePingResult.h"

namespace liboculus {

  /// Abstract base class for all SonarPlayers (files containing of Sonar data)
  class SonarPlayerBase {
  public:
    SonarPlayerBase();
    virtual ~SonarPlayerBase();

    virtual bool open( const std::string &filename );
    virtual bool isOpen( ) const { return _input.is_open(); }

    virtual bool eof() const { return _input.eof(); }
    virtual void rewind() { _input.seekg(0); }

    virtual std::shared_ptr<SimplePingResult> nextPing() = 0;

    /// Static function will attempt to auto-detect the format of the
    /// supplied file and return an appropriate RawSonarPlayer or
    /// GpmfSonarPlayer object.   Will never return a OculusSonarPlayer
    static std::shared_ptr<SonarPlayerBase> OpenFile( const std::string &filename );

  protected:

    std::ifstream _input;
  };

  /// Reads files which contain raw bitstreams of sonar packets.
  ///
  ///
  class RawSonarPlayer : public SonarPlayerBase {
  public:
    RawSonarPlayer();
    virtual ~RawSonarPlayer();

    std::shared_ptr<MessageBuffer> nextPacket();
    virtual std::shared_ptr<SimplePingResult> nextPing();

  private:
  };


  /// Reads sonar files recorded by Oculus Viewer provided by BlueprintSubsea
  /// Does not actually work ... we don't have the format spec for the
  /// messages found in the saved files.
  ///
  ///
  class OculusSonarPlayer : public SonarPlayerBase {
  public:
    OculusSonarPlayer();
    virtual ~OculusSonarPlayer();

    //char *nextPacket();
    virtual std::shared_ptr<SimplePingResult> nextPing();

  private:
  };


}
