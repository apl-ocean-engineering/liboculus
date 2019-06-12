
#include <fstream>

#include "Oculus/Oculus.h"

#include "g3log/g3log.hpp"

#include "liboculus/DataTypes.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/SonarPlayer.h"

namespace liboculus {

using namespace std;

  SonarPlayer::SonarPlayer()
    {;}

  bool SonarPlayer::open( const std::string &filename ) {
      _input.open( filename, ios_base::binary | ios_base::in );

      return _input.is_open();
  }

  char *SonarPlayer::nextPacket() {

    // Advance to the next header byte (actually LSB of header since we're little-endian)
    while( _input.peek() != 0x53 ) {
      char c;
      _input.get(c);
      if( _input.eof() ) return nullptr;
    }

    // Read header
    MessageHeader header;
    _input.read( (char *)&(header.hdr), sizeof(OculusMessageHeader) );

    if( !header.validate() ) {
      LOG(WARNING) << "Incoming header invalid";
      return nullptr;
    }

    char *data = new char[sizeof(OculusMessageHeader) + header.hdr.payloadSize];
    memcpy( data, (void *)&(header.hdr), sizeof(OculusMessageHeader) );
    _input.read( (char *)(&data[sizeof(OculusMessageHeader)]), header.hdr.payloadSize);

    return data;
  }



  std::shared_ptr<SimplePingResult> SonarPlayer::nextPing() {
    char *data = nullptr;
    while( (data = nextPacket()) != nullptr ) {

      MessageHeader header(data);

      if( header.msgId() == messageSimplePingResult ) {
        return std::shared_ptr<SimplePingResult>( new SimplePingResult( data ) );
      } else {
        LOG(DEBUG) << "Skipping " << MessageTypeToString( header.msgId() );
      }

    }

    return std::shared_ptr<SimplePingResult>(nullptr);
  }

}
