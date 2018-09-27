#pragma once

#include <memory>

#include <libg3logger/g3logger.h>

#include "Oculus/Oculus.h"

#include "DataTypes.h"

namespace liboculus {

  class MessageHeader {
  public:
    MessageHeader()
      : _valid(false)
      {
        memset( (void *)&hdr, 0, sizeof(hdr) );
      }

      uint16_t msgId() const { return hdr.msgId; }
      bool valid() const { return _valid; }

      bool validate() {
        _valid = false;

        LOG(DEBUG) <<    "   Oculus Id: 0x" << std::hex << hdr.oculusId;
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


  class BearingData {
  public:
    BearingData()
      : _set(false)
      {}


    short at( unsigned int i ) {
      CHECK( i < _numBeams ) << "Requested beam " << i << " out of range";

      return _ptr[i];
    }

    void set( void *ptr, uint16_t numBeams ) {
      _set = true;

      _ptr = (short *)ptr;
      _numBeams = numBeams;

      LOG(DEBUG) << "Loaded " << _numBeams << " bearings";

      // for(unsigned int i = 0; i < _numBeams; ++i)
      //   LOG(DEBUG) << i << " : " << at(i);
    }

private:
    bool _set;
    short *_ptr;
    uint16_t _numBeams;
  };

  class ImageData {
  public:
    ImageData()
      : _set(false)
      {}

    void set( void *ptr, uint16_t numRanges, uint16_t numBeams, uint8_t bytesPerDatum ) {
      _set = true;

      _ptr = ptr;
      _numRanges = numRanges;
      _numBeams = numBeams;
      _dataSz = bytesPerDatum;

      LOG(DEBUG) << "Loaded " << _numRanges << " x " << _numBeams << " imaging data";

       for(unsigned int i = 0; i < 10; ++i)
          LOG(DEBUG) << i << " : " << std::hex << static_cast<int>(((uint8_t *)&_ptr)[i]);
    }

private:
    bool _set;
    void *_ptr;
    uint16_t _numRanges, _numBeams;
    uint8_t _dataSz;

  };



  class SimplePingResult {
    friend class DataRx;

  public:
    SimplePingResult( const MessageHeader &hdr )
      :  _valid(false), _msg(), _data( nullptr ), _bearings(), _image()
    {
      memcpy( (void *)&_msg, (void *)&(hdr.hdr), sizeof( OculusMessageHeader) );

      if( hdr.valid() ) {
        LOG(DEBUG) << "Initializing header of length " << netHdrLen();
        memset( hdrPtr(), 0, netHdrLen());

        LOG(DEBUG) << "    .. and data array of " << dataLen();
        _data.reset( new char[dataLen()] );
      }
    }

    ~SimplePingResult() {}

    void *hdrPtr()  { return reinterpret_cast<unsigned char*>(&_msg)+sizeof(OculusMessageHeader); }
    void *dataPtr() { return _data.get(); }
    void *imagePtr() { return reinterpret_cast<unsigned char*>(_data.get())+sizeof(_msg.imageOffset)-sizeof(OculusSimplePingResult); }

    bool validate() {
      _valid = false;

      LOG(DEBUG) << "     Mode: " << (int)_msg.fireMessage.masterMode;
      LOG(DEBUG) << "Ping rate: " << _msg.fireMessage.pingRate;

      LOG(DEBUG) << "  Ping ID: " << _msg.pingId;
      LOG(DEBUG) << "   Status: " << _msg.status;

      LOG(DEBUG) << "Frequency: " << _msg.frequency;
      LOG(DEBUG) << "Temperature: " << _msg.temperature;
      LOG(DEBUG) << " Pressure: " << _msg.pressure;
      LOG(DEBUG) << "Spd of Sound: " << _msg.speedOfSoundUsed;
      LOG(DEBUG) << "Range res: " << _msg.rangeResolution << " m";


      LOG(DEBUG) << "Num range: " << _msg.nRanges;
      LOG(DEBUG) << "Num beams: " << _msg.nBeams;

      LOG(DEBUG) << "  Image size: " << _msg.imageSize;
      LOG(DEBUG) << "Image offset: " << _msg.imageOffset;
      LOG(DEBUG) << "   Data size: " << _msg.dataSize;
      LOG(DEBUG) << "Message size: " << _msg.messageSize;

      LOG_IF(WARNING, _msg.messageSize != (dataLen() + sizeof(OculusSimplePingResult)) ) << _msg.messageSize << " != " << (dataLen() + sizeof(OculusSimplePingResult));

      size_t expectedImageSize = DataSize(_msg.dataSize) * _msg.nRanges * _msg.nBeams;

      if( _msg.imageSize != expectedImageSize ) {
        LOG(WARNING) << "ImageSize size in header " << _msg.imageSize << " does not match expected data size of " << expectedImageSize;
        return _valid;
      }

      size_t totalSize = expectedImageSize + _msg.imageOffset;
      if( _msg.messageSize != totalSize ) {
        LOG(WARNING) << "Message size " << _msg.messageSize << " does not match expected message size of " << totalSize;
        return _valid;
      }

      _bearings.set( dataPtr(), _msg.nBeams );
      _image.set( imagePtr(), _msg.nRanges, _msg.nBeams, DataSize(_msg.dataSize) );

      _valid = true;
      return _valid;
    }

    size_t netHdrLen() const { return sizeof(OculusSimplePingResult) - sizeof(OculusMessageHeader); }
    size_t dataLen() const { return _msg.fireMessage.head.payloadSize - netHdrLen(); }

  private:

    bool _valid;
    OculusSimplePingResult _msg;
    std::unique_ptr<char> _data;

    BearingData _bearings;
    ImageData _image;


  };

}
