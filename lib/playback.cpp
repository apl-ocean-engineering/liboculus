
#include <fstream>

#include "Oculus/Oculus.h"

#include "libg3logger/g3logger.h"

#include "liboculus/SimplePingResult.h"

namespace liboculus {

using namespace std;

int playbackSonarFile( const std::string &filename ) {

  ifstream input( filename, ios_base::binary | ios_base::in );

  if( !input.is_open() ) {
    LOG(WARNING) << "Unable to open sonar file " << filename;
    return -1;
  }

  while( !input.eof() ) {

    // Advance to the next header byte
    while( input.peek() != 0x4f ) {
      char c;
      input.get(c);
      if( input.eof() ) return 0;
    }

    // Read header
    MessageHeader header;
    input.read( (char *)&(header.hdr), sizeof(OculusMessageHeader) );


    if( header.validate() ) {

      if( header.msgId() == messageSimplePingResult ) {

        // Read rest of message

        SimplePingResult ping( header );

        input.read( (char *)ping.ptrAfterHeader(), ping.hdr()->payloadSize );

        ping.validate();

      } else if ( header.msgId() == messageLogs && header.hdr.payloadSize > 0 ) {

        char *logMsg = new char[header.hdr.payloadSize + 1];

        input.get( logMsg, header.hdr.payloadSize);

        LOG(INFO) << logMsg;

        } else {

          LOG(INFO) << "Unknown message type, skipping " << header.hdr.payloadSize << " bytes";
          input.seekg( header.hdr.payloadSize, std::ios_base::cur );
        }

    } else {
      LOG(WARNING) << "Incoming header invalid";
    }



  }



  return 0;
}

}
