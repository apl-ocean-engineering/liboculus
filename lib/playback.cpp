
#include <fstream>

#include "Oculus/Oculus.h"

#include "libg3logger/g3logger.h"

#include "liboculus/DataTypes.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/playback.h"

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






int playbackSonarFile( const std::string &filename ) {
  SonarPlayer player;
  if( !player.open(filename) ) {
    LOG(INFO) << "Failed to open " << filename;
    return -1;
  }

  std::shared_ptr<SimplePingResult> ping( player.nextPing() );
  while( ping.get() != nullptr ) {
    ping->validate();
    ping = player.nextPing();
  }

  return 0;
}


}


//
//   ifstream input( filename, ios_base::binary | ios_base::in );
//
//   if( !input.is_open() ) {
//     LOG(WARNING) << "Unable to open sonar file " << filename;
//     return -1;
//   }
//
//   while( !input.eof() ) {
//
//     // Advance to the next header byte (actually LSB of header since we're little-endian)
//     while( input.peek() != 0x53 ) {
//       char c;
//       input.get(c);
//       if( input.eof() ) return 0;
//     }
//
//     // Read header
//     MessageHeader header;
//     input.read( (char *)&(header.hdr), sizeof(OculusMessageHeader) );
//
//
//     if( header.validate() ) {
//
//       if( header.msgId() == messageSimplePingResult ) {
//
//         // Read rest of message
//
//         SimplePingResult ping( header );
//
//         input.read( (char *)ping.ptrAfterHeader(), ping.hdr()->payloadSize );
//
//         ping.validate();
//
//       } else if ( header.msgId() == messageLogs && header.hdr.payloadSize > 0 ) {
//
//         char *logMsg = new char[header.hdr.payloadSize + 1];
//
//         input.get( logMsg, header.hdr.payloadSize);
//
//         LOG(INFO) << logMsg;
//
//         } else {
//
//           LOG(INFO) << "Unknown message type, skipping " << header.hdr.payloadSize << " bytes";
//           input.seekg( header.hdr.payloadSize, std::ios_base::cur );
//         }
//
//     } else {
//       LOG(WARNING) << "Incoming header invalid";
//     }
//
//
//
//   }
//
//
//
//   return 0;
// }
//
// }
