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
#include "liboculus/SimpleFireMessage.h"
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

template <typename Ping_t>
class SimplePingResult : public SimpleFireMessage<typename Ping_t::FireMsg_t> {
 public:
  typedef SimpleFireMessage<typename Ping_t::FireMsg_t> SimpleFireMsg_t;
  typedef GainData<float> GainData_t;

  SimplePingResult() = default;
  SimplePingResult(const SimplePingResult &other) = default;

  explicit SimplePingResult(const std::shared_ptr<ByteVector> &buffer);

  ~SimplePingResult() {}

  const Ping_t *ping() const;

  const BearingData &bearings() const { return _bearings; }
  const GainData_t &gains() const     { return _gains; }
  const ImageData &image() const      { return _image; }

  uint8_t dataSize() const { return SizeOfDataSize(ping()->dataSize); }

  bool valid() const override;
  void dump() const override;

 private:
  // Objects which create OOI overlays the _buffer for  easier interpretation
  BearingData _bearings;

  GainData_t _gains;
  ImageData _image;
};  // class SimplePingResult


typedef SimplePingResult<OculusSimplePingResult> SimplePingResultV1;
typedef SimplePingResult<OculusSimplePingResult2> SimplePingResultV2;


template<typename Ping_t>
SimplePingResult<Ping_t>::SimplePingResult(const std::shared_ptr<ByteVector> &buffer)
  : SimpleFireMsg_t(buffer),
    _bearings(),
    _gains(),
    _image() {
    assert(buffer->size() >= sizeof(Ping_t));

  // Bearing data is packed into an array of shorts at the end of the
  // OculusSimpleFireMessage
  const int16_t *bearingData = reinterpret_cast<const short*>(buffer->data() + sizeof(OculusSimplePingResult));
  _bearings = BearingData(bearingData, this->ping()->nBeams);

  const uint8_t *imageData = reinterpret_cast<const uint8_t*>(buffer->data() + ping()->imageOffset);

  if (this->flags().getSendGain()) { 
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

template<typename Ping_t>
const Ping_t *SimplePingResult<Ping_t>::ping() const {
  return reinterpret_cast<const Ping_t *>(this->buffer()->data());
}

template<typename Ping_t>
bool SimplePingResult<Ping_t>::valid() const {
  if (this->buffer()->size() < sizeof(OculusMessageHeader)) return false;
  if (this->buffer()->size() < this->packetSize()) return false;

  if (!MessageHeader::valid()) {
    LOG(WARNING) << "Header not valid";
    return false;
  }

  int num_pixels = ping()->nRanges * ping()->nBeams;
  size_t expected_size = SizeOfDataSize(ping()->dataSize) * num_pixels;

  if (this->flags().getSendGain()) {
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

template<typename Ping_t>
void SimplePingResult<Ping_t>::dump() const {
  SimpleFireMsg_t::dump();

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
  LOG(DEBUG) << "Message size: " << this->ping()->messageSize;
  LOG(DEBUG) << "--------------";
}

}  // namespace liboculus
