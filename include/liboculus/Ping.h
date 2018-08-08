#pragma once

#include <memory>

#include <libg3logger/g3logger.h>

#include "Oculus/Oculus.h"

namespace liboculus {

  class Ping {
    friend class SonarClient;

  public:
    Ping()
      :  _valid(false), _msg(), _data( nullptr )
    {;}

    ~Ping() {}

    uint16_t msgId() const { return _msg.fireMessage.head.msgId; }

    bool validateOculusMessageHeader() {
      _valid = false;

      if( _msg.fireMessage.head.oculusId != 0x4f53 ) return false;

      LOG(DEBUG) << "      Msg id: " << msgId();
      LOG(DEBUG) << "Payload size: " << _msg.fireMessage.head.payloadSize;

      _valid = true;

      return _valid;
    }

    bool validateOculusSimplePingResult() {

      LOG(DEBUG) << "Ping ID: " << _msg.pingId;
      LOG(DEBUG) << " Status: " << std::hex << _msg.status << std::dec;

      LOG(DEBUG) << "Num range: " << _msg.nRanges;
      LOG(DEBUG) << "Num beams: " << _msg.nBeams;

      LOG(DEBUG) << "  Image size: " << _msg.imageSize;
      LOG(DEBUG) << "   Data size: " << _msg.dataSize;
      LOG(DEBUG) << "Message size: " << _msg.messageSize;

      _data.reset( new char[_msg.fireMessage.head.payloadSize] );

      return _valid;
    }


//      _data.reset( new char[_msg.messageSize]);


    size_t dataLen() const {  if( _valid ) { return _msg.fireMessage.head.payloadSize + sizeof(OculusMessageHeader) - sizeof(OculusSimplePingResult); } else { return 0; } }

  private:

    bool _valid;
    OculusSimplePingResult _msg;
    std::unique_ptr<char> _data;


  };

}
