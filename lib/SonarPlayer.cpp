
#include <fstream>

#include "Oculus/Oculus.h"

#include "g3log/g3log.hpp"

#include "liboculus/DataTypes.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/SonarPlayer.h"
#include "liboculus/GpmfSonarPlayer.h"

namespace liboculus {

  using namespace std;

  /// Static function which automatically detects file type
  shared_ptr<SonarPlayerBase> SonarPlayerBase::OpenFile( const string &filename ) {
    std::ifstream f( filename );

    if( !f.is_open() ) return nullptr;

    char c;
    f.get(c);
    if( c == 0x44 ) {

      char d;
      f.get(d);

      if( d == 0x45 ) {
        LOG(INFO) << "I think this is an GPMF file.";
        return   shared_ptr<SonarPlayerBase>(new GPMFSonarPlayer());
      }

      LOG(INFO) << "I think this is an Oculus client file.";
      return   shared_ptr<SonarPlayerBase>(new OculusSonarPlayer());
    } else if( c == 0x53 ) {
      LOG(INFO) << "I think this is an raw sonar data.";
      return   shared_ptr<SonarPlayerBase>(new RawSonarPlayer());
    }

    return nullptr;
  }


  //--- SonarPlayerBase

  SonarPlayerBase::SonarPlayerBase()
    {;}

  SonarPlayerBase::~SonarPlayerBase()
    {;}


  bool SonarPlayerBase::open( const std::string &filename ) {
      _input.open( filename, ios_base::binary | ios_base::in );

      return _input.is_open();
  }



  //--- RawSonarPlayer --

  RawSonarPlayer::RawSonarPlayer()
    : SonarPlayerBase()
    {;}

  RawSonarPlayer::~RawSonarPlayer()
    {;}

  std::shared_ptr<MessageBuffer> RawSonarPlayer::nextPacket() {

    // Advance to the next header byte (actually LSB of header since we're little-endian)
    while( _input.peek() != 0x53 ) {
      char c;
      _input.get(c);
      if( _input.eof() ) return nullptr;
    }

    // Read header
    std::shared_ptr<MessageBuffer> buffer( new MessageBuffer() );
    _input.read( buffer->ptr(), sizeof(OculusMessageHeader) );

    MessageHeader header( buffer );
    if( !header.valid() ) {
      LOG(WARNING) << "Incoming header invalid";
      return nullptr;
    }

    buffer->expandForPayload();

    // char *data = new char[sizeof(OculusMessageHeader) + header.hdr.payloadSize];
    // memcpy( data, (void *)&(header.hdr), sizeof(OculusMessageHeader) );
    _input.read( buffer->payloadPtr(), buffer->payloadSize() );

    return buffer;
  }



  std::shared_ptr<SimplePingResult> RawSonarPlayer::nextPing() {
    shared_ptr<MessageBuffer> data;
    while( bool(data = nextPacket()) ) {

      MessageHeader header(data);

      if( header.msgId() == messageSimplePingResult ) {
        return std::shared_ptr<SimplePingResult>( new SimplePingResult( data ) );
      } else {
        LOG(DEBUG) << "Skipping message of type " << MessageTypeToString( header.msgId() );
      }

    }

    return std::shared_ptr<SimplePingResult>(nullptr);
  }

  //--- OculusSonarPlayer --

  OculusSonarPlayer::OculusSonarPlayer()
    : SonarPlayerBase()
    {;}

  OculusSonarPlayer::~OculusSonarPlayer()
    {;}

  std::shared_ptr<SimplePingResult> OculusSonarPlayer::nextPing() {
    //
    // Ended up not needing to implement.  The client software provides by
    /// Blueprint records messagePingResult messages, not simplePingResults.
    // We don't have the format for messagePingResult, so we can't parse them
    //

    return std::shared_ptr<SimplePingResult>(nullptr);
  }


}
