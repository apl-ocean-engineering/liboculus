#pragma once

#include <memory>
#include <vector>
#include <string.h>

#include <g3log/g3log.hpp>

#include "Oculus/Oculus.h"

#include "DataTypes.h"

namespace liboculus {

  using std::shared_ptr;
  using std::vector;


  class MessageBuffer {
  public:

    MessageBuffer();
    MessageBuffer( const char *data, size_t len );
    MessageBuffer( const std::vector<char> &vec );


    ~MessageBuffer();

    char *ptr();
    char *headerPtr() { return ptr(); }

    char *dataPtr();

    unsigned int size() const;
    unsigned int payloadSize() const;

    bool expandForPayload();

  protected:

    std::vector<char> _buf;

  };



  class MessageHeader {
  public:
    MessageHeader() = delete;
    MessageHeader( const MessageHeader & ) = delete;

    // MessageHeader()
    //   {
    //     //memset( (void *)&hdr, 0, sizeof(hdr) );
    //   }

    MessageHeader( const shared_ptr<MessageBuffer> &buffer )
      : _buffer( buffer )
      { ; }

    virtual ~MessageHeader()
      {;}

    // Convenience accessors
    OculusMessageType msgId() const       { return static_cast<OculusMessageType>(hdr()->msgId); }
    uint16_t          oculusId() const    { return hdr()->oculusId; }
    uint16_t          srcDeviceId() const { return hdr()->srcDeviceId; }
    uint16_t          dstDeviceId() const { return hdr()->dstDeviceId; }
    uint16_t          msgVersion() const  { return hdr()->msgVersion; }
    uint32_t          payloadSize() const { return hdr()->payloadSize; }


    const OculusMessageHeader *hdr() const { return reinterpret_cast<OculusMessageHeader *>(_buffer->headerPtr()); }

    virtual bool valid() const {
      if( hdr()->oculusId != 0x4f53 ) return false;

      return true;
    }

    std::shared_ptr<MessageBuffer> buffer() { return _buffer; }
    const std::shared_ptr<MessageBuffer> &buffer() const { return _buffer; }


    void dump() const {
      LOG(DEBUG) <<    "   Oculus Id: 0x" << std::hex << hdr()->oculusId;
      LOG(DEBUG) << "      Msg id: 0x" << std::hex << static_cast<uint16_t>(msgId());
      LOG(DEBUG) << "      Dst ID: " << std::hex << hdr()->dstDeviceId;
      LOG(DEBUG) << "      Src ID: " << std::hex << hdr()->srcDeviceId;
      LOG(DEBUG) << "Payload size: " << hdr()->payloadSize << " bytes";
    }

  protected:

    std::shared_ptr<MessageBuffer> _buffer;

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
  class SimplePingResult : public MessageHeader {
    friend class DataRx;

  public:

    SimplePingResult() = delete;
    SimplePingResult( const SimplePingResult & ) = delete;

    SimplePingResult( const shared_ptr<MessageBuffer> &buffer )
      :  MessageHeader(buffer),  _bearings(), _image()
    {
//       //memcpy( (void *)&_msg,
//
//       if( hdr.valid() ) {
// //        LOG(DEBUG) << "Initializing header of length " << netHdrLen();
// //        memset( hdrPtr(), 0, netHdrLen());
//
//         _dataSize = sizeof(OculusMessageHeader) + hdr.hdr.payloadSize;
//         LOG(DEBUG) << "Creating message buffer of " << _dataSize;
//
//         // alloc a 4-byte aligned buffer
//         size_t alignSize = ((_dataSize + 3) >> 2) << 2;
//         _data.reset( new uint8_t[alignSize] );
//
//         memcpy( (void *)_data.get(), (void *)&(hdr.hdr), sizeof( OculusMessageHeader) );
//       }
    }

    // SimplePingResult( char *data )
    //   :  _data( nullptr ), _bearings(), _image()
    // {
    //   //_data.reset( (unsigned char*)data );
    //
    //   OculusMessageHeader *hdr = reinterpret_cast<OculusMessageHeader *>(data);
    //
    //   _dataSize = sizeof(OculusMessageHeader) + hdr->payloadSize;
    //
    //   // alloc a 4-byte aligned buffer
    //   size_t alignSize = ((_dataSize + 3) >> 2) << 2;
    //   _data.reset( new uint8_t[alignSize] );
    //
    //   memcpy( (void *)_data.get(), (void *)data, _dataSize );
    // }


    virtual ~SimplePingResult() {}

    // void *data()                  { return reinterpret_cast<unsigned char*>(_data.get()); }
    // const size_t dataSize() const { return _dataSize; }

    OculusSimplePingResult *ping()             { return reinterpret_cast<OculusSimplePingResult *>( _buffer->ptr() ); }
    const OculusSimplePingResult *ping() const { return reinterpret_cast<const OculusSimplePingResult *>( _buffer->ptr() ); }

    OculusSimpleFireMessage *fireMsg() { return reinterpret_cast<OculusSimpleFireMessage *>( _buffer->ptr() ); }
    const OculusSimpleFireMessage *fireMsg() const { return reinterpret_cast<const OculusSimpleFireMessage *>( _buffer->ptr() ); }
    //OculusMessageHeader *hdr()     { return &(ping()->fireMessage.head); }

    // MessageHeader header()             { return MessageHeader( (const char *)_data.get() ); }
    // const MessageHeader header() const { return MessageHeader( (const char *)_data.get() ); }


    // void *ptrAfterHeader() { return reinterpret_cast<void *>( _data.get() + sizeof(OculusMessageHeader)); }

    const BearingData &bearings() const { return _bearings; }
    const ImageData   &image() const    { return _image; }


    // size_t imageLen const { return header()->payloadSize  }
    //
    // size_t netHdrLen() const { return sizeof(OculusSimplePingResult) - sizeof(OculusMessageHeader); }
    // size_t dataLen() const { return _msg.fireMessage.head.payloadSize - netHdrLen(); }


//    void *hdrPtr()  { return reinterpret_cast<unsigned char*>(&_msg)+sizeof(OculusMessageHeader); }
//    void *dataPtr() { return _data.get(); }
//    void *imagePtr() { return reinterpret_cast<unsigned char*>(_data.get())+sizeof(_msg.imageOffset)-sizeof(OculusSimplePingResult); }

    virtual bool valid() const {
      if( !MessageHeader::valid() ) return false;

      LOG(DEBUG) << "     Mode: " << fireMsg()->masterMode;
      LOG(DEBUG) << "Ping rate: " << fireMsg()->pingRate;

      LOG(DEBUG) << "  Ping ID: " << ping()->pingId;
      LOG(DEBUG) << "   Status: " << ping()->status;

      LOG(DEBUG) << "Frequency: " <<   ping()->frequency;
      LOG(DEBUG) << "Temperature: " << ping()->temperature;
      LOG(DEBUG) << " Pressure: " <<   ping()->pressure;
      LOG(DEBUG) << "Spd of Sound: " << ping()->speedOfSoundUsed;
      LOG(DEBUG) << "Range res: " <<   ping()->rangeResolution << " m";


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
        return false;
      }

      // size_t totalSize = expectedImageSize + _msg.imageOffset;
      // if( _msg.messageSize != totalSize ) {
      //   LOG(WARNING) << "Message size " << _msg.messageSize << " does not match expected message size of " << totalSize;
      //   return _valid;
      // }

      CHECK( ping()->imageOffset > sizeof(OculusSimplePingResult) );
      return true;
    }

    bool update() {

      if( !valid() ) return false;

      _bearings.set( _buffer->ptr() + sizeof(OculusSimplePingResult), ping()->nBeams );
      _image.set(    _buffer->ptr() + ping()->imageOffset, ping()->nRanges, ping()->nBeams, ping()->dataSize );

      return true;
    }

  private:

    std::unique_ptr<uint8_t> _data;
    size_t _dataSize;

    // Objects which overlay _data for interpretation
    BearingData _bearings;
    ImageData _image;


  };

}
