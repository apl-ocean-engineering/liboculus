/*
 * Copyright (c) 2017-2020 Aaron Marburg <amarburg@uw.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of University of Washington nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 #pragma once

#include <memory>
#include <string.h>
#include <vector>

#include <g3log/g3log.hpp>

#include "Oculus/Oculus.h"

#include "DataTypes.h"
#include "ImageData.h"
#include "BearingData.h"

namespace liboculus {

using std::shared_ptr;
using std::vector;

class MessageBuffer {
public:
  /// Default constructor creates a buffer large enough for an OculusMessageHeader
  MessageBuffer( int reserve = sizeof(OculusMessageHeader) );

  /// Alternative constructors which initialize from existing data
  /// These take a copy of the data.
  MessageBuffer(const char *data, size_t len);
  MessageBuffer(const std::vector<char> &vec);

  ~MessageBuffer();

  char *ptr() { return _buf.data(); }

  const char *ptr() const { return _buf.data(); }
  const char *headerPtr() { return ptr(); }

  char *payloadPtr();
  const char *payloadPtr() const;

  unsigned int size() const;
  unsigned int payloadSize() const;

  bool expandForPayload();

protected:
  std::vector<char> _buf;
};


class MessageHeader {
public:
  MessageHeader() = delete;
  MessageHeader(const MessageHeader &) = delete;

  MessageHeader(const shared_ptr<MessageBuffer> &buffer)
    : _buffer(buffer) {
    ;
  }

  ~MessageHeader() { ; }

  // Convenience accessors
  OculusMessageType msgId() const {
    return static_cast<OculusMessageType>(hdr()->msgId);
  }
  uint16_t oculusId() const { return hdr()->oculusId; }
  uint16_t srcDeviceId() const { return hdr()->srcDeviceId; }
  uint16_t dstDeviceId() const { return hdr()->dstDeviceId; }
  uint16_t msgVersion() const { return hdr()->msgVersion; }
  uint32_t payloadSize() const { return hdr()->payloadSize; }

  virtual bool valid() const {
    if (hdr()->oculusId != 0x4f53)
      return false;

    return true;
  }

  std::shared_ptr<MessageBuffer> buffer() { return _buffer; }
  const std::shared_ptr<MessageBuffer> &buffer() const { return _buffer; }

  void dump() const {
    LOG(DEBUG) << "   Oculus Id: 0x" << std::hex << oculusId();
    LOG(DEBUG) << "      Msg id: 0x" << std::hex
               << static_cast<uint16_t>(msgId());
    LOG(DEBUG) << "      Dst ID: " << std::hex << dstDeviceId();
    LOG(DEBUG) << "      Src ID: " << std::hex << srcDeviceId();
    LOG(DEBUG) << "Payload size: " << payloadSize() << " bytes";
  }

protected:
  const OculusMessageHeader *hdr() const {
    return reinterpret_cast<const OculusMessageHeader *>(_buffer->headerPtr());
  }

  std::shared_ptr<MessageBuffer> _buffer;

};


// A single OculusSimplePingResult (msg) is actually three nested structs:
//   OculusMessageHeader     (as msg.fireMessage.head)
//   OculusSimpleFireMessage (as msg.fireMessage)
//   then the rest of OculusSimplePingResult
class SimplePingResult {
  friend class DataRx;

public:
  SimplePingResult() = delete;
  SimplePingResult(const SimplePingResult &) = delete;

  SimplePingResult(const shared_ptr<MessageHeader> &header)
      : _header(header), _bearings(), _image() {
    // TODO, this could be done through a constructor
    _bearings.set(buffer()->ptr() + sizeof(OculusSimplePingResult),
                  oculusPing()->nBeams);
    _image.set(buffer()->ptr() + oculusPing()->imageOffset, oculusPing()->nRanges,
               oculusPing()->nBeams, oculusPing()->dataSize);
  }

  ~SimplePingResult() {}

  // Unpack the contents
  const shared_ptr<MessageBuffer> buffer( void ) const { return header()->buffer(); }
  const shared_ptr<MessageHeader> header( void ) const { return _header; }

  /// Cast back to the original Oculus API message(s)
  // OculusSimplePingResult *oculusPing() {
  //   return reinterpret_cast<OculusSimplePingResult *>(_buffer->ptr());
  // }
  const OculusSimpleFireMessage *oculusFireMsg() const {
    return reinterpret_cast<const OculusSimpleFireMessage *>(buffer()->ptr());
  }

  const OculusSimplePingResult *oculusPing() const {
    return reinterpret_cast<const OculusSimplePingResult *>(buffer()->ptr());
  }

  const BearingData &bearings() const { return _bearings; }
  const ImageData &image() const { return _image; }

  virtual bool valid() const {
    if (!_header->valid())
      return false;

    size_t expectedImageSize =
        SizeOfDataSize(oculusPing()->dataSize) * oculusPing()->nRanges * oculusPing()->nBeams;

    if (oculusPing()->imageSize != expectedImageSize) {
      LOG(WARNING) << "ImageSize size in header " << oculusPing()->imageSize
                   << " does not match expected data size of "
                   << expectedImageSize;
      return false;
    }

    // size_t totalSize = expectedImageSize + _msg.imageOffset;
    // if( _msg.messageSize != totalSize ) {
    //   LOG(WARNING) << "Message size " << _msg.messageSize << " does not match
    //   expected message size of " << totalSize; return _valid;
    // }

    CHECK(oculusPing()->imageOffset > sizeof(OculusSimplePingResult));
    return true;
  }

  void dump() const {
    LOG(DEBUG) << "--------------";
    LOG(DEBUG) << "        Mode: " << oculusFireMsg()->masterMode;
    LOG(DEBUG) << "   Ping rate: " << oculusFireMsg()->pingRate;

    LOG(DEBUG) << "     Ping ID: " << oculusPing()->pingId;
    LOG(DEBUG) << "      Status: " << oculusPing()->status;
    LOG(DEBUG) << "   Ping start time: " << oculusPing()->pingStartTime;

    LOG(DEBUG) << "   Frequency: " << oculusPing()->frequency;
    LOG(DEBUG) << " Temperature: " << oculusPing()->temperature;
    LOG(DEBUG) << "    Pressure: " << oculusPing()->pressure;
    LOG(DEBUG) << "Spd of Sound: " << oculusPing()->speedOfSoundUsed;
    LOG(DEBUG) << "   Range res: " << oculusPing()->rangeResolution << " m";

    LOG(DEBUG) << "   Num range: " << oculusPing()->nRanges;
    LOG(DEBUG) << "   Num beams: " << oculusPing()->nBeams;

    LOG(DEBUG) << "  Image size: " << oculusPing()->imageSize;
    LOG(DEBUG) << "Image offset: " << oculusPing()->imageOffset;
    LOG(DEBUG) << "   Data size: " << DataSizeToString(oculusPing()->dataSize);
    LOG(DEBUG) << "Message size: " << oculusPing()->messageSize;
    LOG(DEBUG) << "--------------";
  }

private:
  shared_ptr<MessageHeader> _header;

  // Objects which overlay _data for interpretation
  BearingData _bearings;
  ImageData _image;
};

} // namespace liboculus
