#pragma once

#include <memory>

#include <libg3logger/g3logger.h>

#include "Oculus/Oculus.h"

namespace liboculus {

  class MessageHeader {
  public:
    MessageHeader()
      : _valid(false)
      {
        memset( (void *)&hdr, 0, sizeof(hdr) );
      }

      uint16_t msgId() const { return hdr.msgId; }

      bool validate() {
        _valid = false;

        LOG(DEBUG) <<    "Oculus Id: 0x" << std::hex << hdr.oculusId;

        if( hdr.oculusId != 0x4f53 ) return false;

        LOG(DEBUG) << "      Msg id: 0x" << std::hex << msgId();
        LOG(DEBUG) << "      Dst ID: " << std::hex << hdr.dstDeviceId;
        LOG(DEBUG) << "      Src ID: " << std::hex << hdr.srcDeviceId;
        LOG(DEBUG) << "Payload size: " << hdr.payloadSize << " bytes";

        _valid = true;

        return _valid;
      }

      bool _valid;
      OculusMessageHeader hdr;

  };



  class SimplePingResult {
    friend class SonarClient;

  public:
    SimplePingResult( const MessageHeader &hdr )
      :  _valid(false), _msg(), _data( nullptr )
    {
      memcpy( (void *)&_msg, (void *)&(hdr.hdr), sizeof( OculusMessageHeader) );
      memset( hdrPtr(), 0, netHdrLen());

      _data.reset( new char[dataLen()] );
    }

    ~SimplePingResult() {}

    void *hdrPtr()  { return (void *)(&_msg + sizeof(OculusMessageHeader)); }
    void *dataPtr() { return _data.get(); }

    bool validate() {

      LOG(DEBUG) << "Do validate.";

      LOG(DEBUG) << "     Mode: " << (int)_msg.fireMessage.masterMode;
      LOG(DEBUG) << "Ping rate: " << _msg.fireMessage.pingRate;

      LOG(DEBUG) << "  Ping ID: " << _msg.pingId;
      LOG(DEBUG) << "   Status: " << _msg.status;

      LOG(DEBUG) << "Num range: " << _msg.nRanges;
      LOG(DEBUG) << "Num beams: " << _msg.nBeams;

      LOG(DEBUG) << "  Image size: " << _msg.imageSize;
      LOG(DEBUG) << "   Data size: " << _msg.dataSize;
      LOG(DEBUG) << "Message size: " << _msg.messageSize;

      _valid = true;

      return _valid;
    }

    size_t netHdrLen() const { return sizeof(OculusSimplePingResult) - sizeof(OculusMessageHeader); }
    size_t dataLen() const { return _msg.fireMessage.head.payloadSize - netHdrLen(); }

  private:

    bool _valid;
    OculusSimplePingResult _msg;
    std::unique_ptr<char> _data;


  };

}
