#pragma once

#include "liboculus/SimplePingResult.h"

namespace liboculus {


  MessageBuffer::MessageBuffer()
    : _buf( sizeof( OculusMessageHeader ), 0 )
    {;}

  MessageBuffer::MessageBuffer( const char *data, size_t len )
    : _buf( len )
    {
      memcpy( _buf.data(), data, len );
    }

  MessageBuffer::MessageBuffer( const std::vector<char> &vec )
    : _buf( vec )
    {;}

  MessageBuffer::~MessageBuffer()
    {;}

  char *MessageBuffer::ptr()
    { return _buf.data(); }

  char *MessageBuffer::dataPtr()
    { return _buf.data() + sizeof(OculusMessageHeader); }

  unsigned int MessageBuffer::size() const
    { return _buf.size(); }

  unsigned int MessageBuffer::payloadSize() const
    { return _buf.size() - sizeof(OculusMessageHeader); }

  bool MessageBuffer::expandForPayload()
    {
      OculusMessageHeader *hdr = reinterpret_cast<OculusMessageHeader *>(_buf.data());
      if( hdr->oculusId != 0x4f53 ) return false;

      // alloc a 4-byte aligned buffer
      const size_t dataSize = sizeof(OculusMessageHeader) + hdr->payloadSize;
      const size_t alignSize = ((dataSize + 3) >> 2) << 2;
      _buf.resize( alignSize );

      return true;
    }



}
