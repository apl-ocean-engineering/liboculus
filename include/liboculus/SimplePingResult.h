/*
 * Copyright (c) 2017-2022 University of Washington
 * Author: Aaron Marburg <amarburg@uw.edu>
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
#include <string>
#include <vector>
#include <cassert>

#include <g3log/g3log.hpp>

#include "liboculus/BearingData.h"
#include "liboculus/GainData.h"
#include "liboculus/DataTypes.h"
#include "liboculus/ImageData.h"

#include "Oculus/Oculus.h"
#include "liboculus/MessageHeader.h"
#include "liboculus/SonarConfiguration.h"

namespace liboculus {

using std::shared_ptr;
using std::vector;

// A single OculusSimplePingResult (msg) is actually three nested structs:
//   OculusMessageHeader     (as msg.fireMessage.head)
//   OculusSimpleFireMessage (as msg.fireMessage)
//   then the rest of OculusSimplePingResult

// Conveniently the OculusSimplePingResult and OculusSimplePingResult2
// structs have the same fields, but a different structure

struct PingV1 {
  typedef OculusSimplePingResult  Ping_t;
  typedef OculusSimpleFireMessage FireMsg_t;
};

struct PingV2 {
  typedef OculusSimplePingResult2  Ping_t;
  typedef OculusSimpleFireMessage2 FireMsg_t;
};


template <typename PingT = PingV1>
class SimplePingResult : public MessageHeader {
 public:
  typedef GainData<int32_t> GainData_t;

  SimplePingResult() = default;
  SimplePingResult(const SimplePingResult &other) = default;

  explicit SimplePingResult(const std::shared_ptr<ByteVector> &buffer);

  ~SimplePingResult() {}

  const typename PingT::FireMsg_t *fireMsg() const {
      return reinterpret_cast<const typename PingT::FireMsg_t *>(_buffer->data());
  }

  const typename PingT::Ping_t *ping() const  {
    return reinterpret_cast<const typename PingT::Ping_t *>(_buffer->data());
  }

  const OculusSimpleFireFlags &flags() const {
    return _flags;
  }

  const BearingData &bearings() const { return _bearings; }
  const GainData_t &gains() const     { return _gains; }
  const ImageData &image() const      { return _image; }

  uint8_t dataSize() const { return SizeOfDataSize(ping()->dataSize); }

  bool valid() const override;
  void dump() const override;

 private:
  OculusSimpleFireFlags _flags;

  // Objects which create OOI overlays the _buffer for  easier interpretation
  BearingData _bearings;

  GainData_t _gains;
  ImageData _image;

};  // class SimplePingResult


typedef SimplePingResult<PingV1> SimplePingResultV1;
typedef SimplePingResult<PingV2> SimplePingResultV2;


template<typename PingT>
SimplePingResult<PingT>::SimplePingResult(const std::shared_ptr<ByteVector> &buffer)
  : MessageHeader(buffer),
  _flags(this->fireMsg()->flags),
  _bearings(),
  _gains(),
  _image() {
  assert(buffer->size() >= sizeof(OculusSimplePingResult));

  // Bearing data is packed into an array of shorts at the end of the
  // OculusSimpleFireMessage
  const int16_t *bearingData = reinterpret_cast<const short*>(buffer->data() + sizeof(OculusSimplePingResult));
  _bearings = BearingData(bearingData, this->ping()->nBeams);

  const uint8_t *imageData = reinterpret_cast<const uint8_t*>(buffer->data() + ping()->imageOffset);

  if (_flags.getSendGain()) { 
    // If sent, the gain is included as the first 4 bytes in each "row" of data 
    const uint16_t offsetBytes = 4;

    // The size of one "row" of data in bytes
    const uint16_t strideBytes = SizeOfDataSize(ping()->dataSize)*this->ping()->nBeams + offsetBytes;
    _image = ImageData(imageData,
                          this->ping()->imageSize,
                          this->ping()->nRanges,
                          this->ping()->nBeams,
                          SizeOfDataSize(this->ping()->dataSize),
                          strideBytes,
                          offsetBytes);

    _gains = GainData_t(reinterpret_cast<const GainData_t::DataType *>(imageData),
                          this->ping()->imageSize,
                          strideBytes, 
                          this->ping()->nRanges);
  } else {
    _image = ImageData(imageData,
                          this->ping()->imageSize,
                          this->ping()->nRanges,
                          this->ping()->nBeams,
                          SizeOfDataSize(this->ping()->dataSize));
  }
}

template<typename PingT>
bool SimplePingResult<PingT>::valid() const {
  if (_buffer->size() < sizeof(OculusMessageHeader)) return false;
  if (_buffer->size() < packetSize()) return false;

  MessageHeader hdr(_buffer);
  if (!hdr.valid()) {
    LOG(WARNING) << "Header not valid";
    return false;
  }

  int num_pixels = ping()->nRanges * ping()->nBeams;
  size_t expected_size = SizeOfDataSize(ping()->dataSize) * num_pixels;

  if (flags().getSendGain()) {
    expected_size += sizeof(uint32_t) * ping()->nRanges;
  }

  if (ping()->imageSize != expected_size) {
    LOG(WARNING) << "ImageSize in header " << ping()->imageSize
                 << " does not match expected data size of "
                 << expected_size;
    return false;
  }

  CHECK(ping()->imageOffset > sizeof(OculusSimplePingResult));
  return true;
}

template<typename PingT>
void SimplePingResult<PingT>::dump() const {
  LOG(DEBUG) << "--------------";
  MessageHeader::dump();
  LOG(DEBUG) << "        Mode: " << FreqModeToString(this->fireMsg()->masterMode);

  const int pingRate = PingRateToHz(this->fireMsg()->pingRate);
  if (pingRate >= 0 ) {
    LOG(DEBUG) << "   Ping rate: " << pingRate;
  } else {
    LOG(DEBUG) << "   Ping rate: (unknown) " << static_cast<int>(this->fireMsg()->pingRate);
  }

  LOG(DEBUG) << "     Ping ID: " << this->ping()->pingId;
  LOG(DEBUG) << "      Status: " << this->ping()->status;
  LOG(DEBUG) << "   Ping start time: " << this->ping()->pingStartTime;

  LOG(DEBUG) << "   Frequency: " << this->ping()->frequency;
  LOG(DEBUG) << " Temperature: " << this->ping()->temperature;
  LOG(DEBUG) << "    Pressure: " << this->ping()->pressure;
  LOG(DEBUG) << "Spd of Sound: " << this->ping()->speedOfSoundUsed;
  LOG(DEBUG) << "   Range res: " << this->ping()->rangeResolution << " m";

  LOG(DEBUG) << "   Num range: " << this->ping()->nRanges;
  LOG(DEBUG) << "   Num beams: " << this->ping()->nBeams;

  LOG(DEBUG) << "  Image size: " << this->ping()->imageSize;
  LOG(DEBUG) << "Image offset: " << this->ping()->imageOffset;
  LOG(DEBUG) << "   Data size: " << DataSizeToString(this->ping()->dataSize);
  LOG(DEBUG) << "   Send gain: " << (this->flags().getSendGain() ? "Yes" : "No");
  LOG(DEBUG) << "Message size: " << this->ping()->messageSize;
  LOG(DEBUG) << "--------------";
}

}  // namespace liboculus
