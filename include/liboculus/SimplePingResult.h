#pragma once

#include <memory>
#include <string.h>

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

    MessageHeader( const char *data )
      : _valid(false)
      {
          memcpy( (void *)&hdr, (void *)data, sizeof(OculusMessageHeader) );
      }

      // Convenience accessors
      OculusMessageType msgId() const { return static_cast<OculusMessageType>(hdr.msgId); }

      bool valid() const { return _valid; }

      bool validate() {
        _valid = false;

        LOG(DEBUG) <<    "   Oculus Id: 0x" << std::hex << hdr.oculusId;
        if( hdr.oculusId != 0x4f53 ) return false;

        LOG(DEBUG) << "      Msg id: 0x" << std::hex << static_cast<uint16_t>(msgId());
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

    // Returns bearing in degrees
    float at( unsigned int i ) const {
      CHECK( i < _numBeams ) << "Requested beam " << i << " out of range";

      return _ptr[i]/100.0;
    }

    void set( void *ptr, uint16_t numBeams ) {
      _set = true;

      _ptr = (short *)ptr;
      _numBeams = numBeams;

      LOG(DEBUG) << "Loaded " << _numBeams << " bearings";

      // for(unsigned int i = 0; i < _numBeams; ++i)
      //   LOG(DEBUG) << i << " : " << _ptr[i];
    }

private:
    bool _set;
    short *_ptr;
    uint16_t _numBeams;
  };

  class ImageData {
  public:
    ImageData()
      : _ptr(nullptr)
      {}

    // TODO.  Deal with non-8-bit data somehow
    uint8_t at( unsigned int bearing, unsigned int range ) const {
      CHECK( _dataSz == 1 ) << "Sorry, can only handle 8-bit data right now";
      if( _ptr == nullptr ) return 0;

      //TODO range check
      const unsigned int index = range * _numBeams + bearing;
      CHECK( index < (_numRanges * _numBeams) );

      return ( (uint8_t *)_ptr)[range * _numBeams + bearing];
    }

    void set( void *ptr, uint16_t numRanges, uint16_t numBeams, DataSizeType bytesPerDatum ) {
      _ptr = ptr;
      _numRanges = numRanges;
      _numBeams = numBeams;
      _dataSz = SizeOfDataSize(bytesPerDatum);

      LOG(DEBUG) << "Loaded " << _numRanges << " x " << _numBeams << " imaging data";

       for(unsigned int i = 0; i < 10; ++i)
          LOG(DEBUG) << i << " : " << std::hex << static_cast<uint16_t>(((uint8_t *)_ptr)[i]);
    }

private:
    void *_ptr;
    uint16_t _numRanges, _numBeams;
    uint8_t _dataSz;

  };



  // A single OculusSimplePingResult (msg) is actually three nested structs:
  //   OculusMessageHeader     (as msg.fireMessage.head)
  //   OculusSimpleFireMessage (as msg.fireMessage)
  //   then the rest of OculusSimplePingResult
  class SimplePingResult {
    friend class DataRx;

  public:
    SimplePingResult( const MessageHeader &hdr )
      :  _valid(false), _data( nullptr ), _bearings(), _image()
    {
      //memcpy( (void *)&_msg,

      if( hdr.valid() ) {
//        LOG(DEBUG) << "Initializing header of length " << netHdrLen();
//        memset( hdrPtr(), 0, netHdrLen());

        _dataSize = sizeof(OculusMessageHeader) + hdr.hdr.payloadSize;
        LOG(DEBUG) << "Creating message buffer of " << _dataSize;

        // alloc a 4-byte aligned buffer
        size_t alignSize = ((_dataSize + 3) >> 2) << 2;
        _data.reset( new uint8_t[alignSize] );

        memcpy( (void *)_data.get(), (void *)&(hdr.hdr), sizeof( OculusMessageHeader) );
      }
    }

    SimplePingResult( char *data )
      :  _valid(false), _data( nullptr ), _bearings(), _image()
    {
      //_data.reset( (unsigned char*)data );

      OculusMessageHeader *hdr = reinterpret_cast<OculusMessageHeader *>(data);

      _dataSize = sizeof(OculusMessageHeader) + hdr->payloadSize;

      // alloc a 4-byte aligned buffer
      size_t alignSize = ((_dataSize + 3) >> 2) << 2;
      _data.reset( new uint8_t[alignSize] );

      memcpy( (void *)_data.get(), (void *)data, _dataSize );
    }


    ~SimplePingResult() {}

    void *data()     { return reinterpret_cast<unsigned char*>(_data.get()); }
    const size_t dataSize() const { return _dataSize; }

    OculusSimplePingResult *ping() { return reinterpret_cast<OculusSimplePingResult *>( _data.get() ); }
    OculusMessageHeader *hdr() { return &(ping()->fireMessage.head); }

    MessageHeader header() { return MessageHeader( (const char *)_data.get() ); }

    void *ptrAfterHeader() { return reinterpret_cast<void *>( _data.get() + sizeof(OculusMessageHeader)); }

    const BearingData &bearings() const { return _bearings; }
    const ImageData   &image() const    { return _image; }


    // size_t imageLen const { return header()->payloadSize  }
    //
    // size_t netHdrLen() const { return sizeof(OculusSimplePingResult) - sizeof(OculusMessageHeader); }
    // size_t dataLen() const { return _msg.fireMessage.head.payloadSize - netHdrLen(); }


//    void *hdrPtr()  { return reinterpret_cast<unsigned char*>(&_msg)+sizeof(OculusMessageHeader); }
//    void *dataPtr() { return _data.get(); }
//    void *imagePtr() { return reinterpret_cast<unsigned char*>(_data.get())+sizeof(_msg.imageOffset)-sizeof(OculusSimplePingResult); }

    bool validate() {
      _valid = false;

      LOG(DEBUG) << "     Mode: " << (int)ping()->fireMessage.masterMode;
      LOG(DEBUG) << "Ping rate: " << ping()->fireMessage.pingRate;

      LOG(DEBUG) << "  Ping ID: " << ping()->pingId;
      LOG(DEBUG) << "   Status: " << ping()->status;

      LOG(DEBUG) << "Frequency: " << ping()->frequency;
      LOG(DEBUG) << "Temperature: " << ping()->temperature;
      LOG(DEBUG) << " Pressure: " << ping()->pressure;
      LOG(DEBUG) << "Spd of Sound: " << ping()->speedOfSoundUsed;
      LOG(DEBUG) << "Range res: " << ping()->rangeResolution << " m";


      LOG(DEBUG) << "Num range: " << ping()->nRanges;
      LOG(DEBUG) << "Num beams: " << ping()->nBeams;

      LOG(DEBUG) << "  Image size: " << ping()->imageSize;
      LOG(DEBUG) << "Image offset: " << ping()->imageOffset;
      LOG(DEBUG) << "   Data size: " << DataSizeToString(ping()->dataSize);
      LOG(DEBUG) << "Message size: " << ping()->messageSize;

      //LOG_IF(WARNING, hdr().messageSize != _dataSize ) << ping()->messageSize << " != " << _dataSize;

      size_t expectedImageSize = SizeOfDataSize(ping()->dataSize) * ping()->nRanges * ping()->nBeams;

      if( ping()->imageSize != expectedImageSize ) {
        LOG(WARNING) << "ImageSize size in header " << ping()->imageSize << " does not match expected data size of " << expectedImageSize;
        return _valid;
      }

      // size_t totalSize = expectedImageSize + _msg.imageOffset;
      // if( _msg.messageSize != totalSize ) {
      //   LOG(WARNING) << "Message size " << _msg.messageSize << " does not match expected message size of " << totalSize;
      //   return _valid;
      // }

      CHECK( ping()->imageOffset > sizeof(OculusSimplePingResult) );

      _bearings.set( _data.get() + sizeof(OculusSimplePingResult), ping()->nBeams );
      _image.set(   _data.get() + ping()->imageOffset, ping()->nRanges, ping()->nBeams, ping()->dataSize );

      _valid = true;
      return _valid;
    }

  private:

    bool _valid;
    std::unique_ptr<uint8_t> _data;
    size_t _dataSize;

    // Objects which overlay _data for interpretation
    BearingData _bearings;
    ImageData _image;


  };

}
