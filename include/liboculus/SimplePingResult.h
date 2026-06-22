/*
 * Copyright (c) 2017-2025 University of Washington
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

#include <cassert>
#include <fmt/format.h>
#include <memory>
#include <string>
#include <vector>

#include "liboculus/BearingData.h"
#include "liboculus/DataTypes.h"
#include "liboculus/GainData.h"
#include "liboculus/ImageData.h"
#include "liboculus/Logger.h"
#include "liboculus/SimpleFireMessage.h"
#include "liboculus/SonarConfiguration.h"
#include "liboculus/thirdparty/Oculus/Oculus.h"

namespace liboculus {

using std::shared_ptr;
using std::vector;

// A single OculusSimplePingResult (msg) is actually three nested structs:
//   OculusMessageHeader     (as msg.fireMessage.head)
//   OculusSimpleFireMessage (as msg.fireMessage)
//   then the rest of OculusSimplePingResult

// Conveniently the OculusSimplePingResult and OculusSimplePingResult2
// structs have the same fields, but a different structure.
// If that weren't the case, we'd use two separate classes instead
// of templates...

template <typename Ping_t>
class SimplePingResult : public SimpleFireMessage<typename Ping_t::FireMsg_t> {
public:
  typedef SimpleFireMessage<typename Ping_t::FireMsg_t> SimpleFireMsg_t;
  typedef GainData<float> GainData_t;

  SimplePingResult() = default;
  SimplePingResult(const SimplePingResult &other) = default;

  explicit SimplePingResult(const std::shared_ptr<ByteVector> &buffer);

  virtual ~SimplePingResult() {}

  const Ping_t *ping() const;

  const BearingData &bearings() const { return _bearings; }
  const GainData_t &gains() const { return _gains; }
  const ImageData &image() const { return _image; }

  uint8_t dataSize() const { return SizeOfDataSize(ping()->dataSize); }

  bool valid() const override;
  std::vector<std::string> dump(std::vector<std::string> &) const override;

private:
  // Objects which create OOI overlays the _buffer for  easier interpretation
  BearingData _bearings;

  GainData_t _gains;
  ImageData _image;
}; // class SimplePingResult

typedef SimplePingResult<OculusSimplePingResult> SimplePingResultV1;
typedef SimplePingResult<OculusSimplePingResult2> SimplePingResultV2;

template <typename Ping_t>
SimplePingResult<Ping_t>::SimplePingResult(
    const std::shared_ptr<ByteVector> &buffer)
    : SimpleFireMsg_t(buffer), _bearings(), _gains(), _image() {
  if (buffer->size() < sizeof(Ping_t)) {
    oclog::error("SimplePingResult: buffer too small ({} < {})",
                 buffer->size(), sizeof(Ping_t));
    return;
  }

  const auto nBeams = this->ping()->nBeams;
  const auto nRanges = this->ping()->nRanges;
  const auto imageOffset = this->ping()->imageOffset;
  const auto imageSize = this->ping()->imageSize;

  oclog::debug(
      "SimplePingResult: beams={} ranges={} imageOffset={} imageSize={} "
      "dataSize={}",
      nBeams, nRanges, imageOffset, imageSize,
      static_cast<int>(this->ping()->dataSize));

  const size_t bearingsOffset = sizeof(Ping_t);
  const size_t bearingsBytes = static_cast<size_t>(nBeams) * sizeof(int16_t);
  if (bearingsOffset + bearingsBytes > buffer->size()) {
    oclog::error(
        "SimplePingResult: bearing data out of range (offset={}, bytes={}, "
        "buffer={})",
        bearingsOffset, bearingsBytes, buffer->size());
    return;
  }

  if (static_cast<size_t>(imageOffset) + imageSize > buffer->size()) {
    oclog::error(
        "SimplePingResult: image data out of range (offset={}, size={}, "
        "buffer={})",
        imageOffset, imageSize, buffer->size());
    return;
  }

  // Bearing data is packed into an array of numBeams shorts immediately
  // the end of the OculusSimplePing struct
  const int16_t *bearingData =
      reinterpret_cast<const short *>(buffer->data() + sizeof(Ping_t));
  _bearings = BearingData(bearingData, this->ping()->nBeams);

  const uint8_t *imageData =
      reinterpret_cast<const uint8_t *>(buffer->data() + ping()->imageOffset);

  if (this->flags().getSendGain()) {
    // If sent, the gain is included as the first 4 bytes in
    // each row of data
    const uint16_t offsetBytes = 4;

    // The size of one "row" of data in bytes
    const uint16_t strideBytes =
        SizeOfDataSize(ping()->dataSize) * this->ping()->nBeams + offsetBytes;
    _image =
        ImageData(imageData, this->ping()->imageSize, this->ping()->nRanges,
                  this->ping()->nBeams, SizeOfDataSize(this->ping()->dataSize),
                  strideBytes, offsetBytes);

    _gains =
        GainData_t(reinterpret_cast<const GainData_t::DataType *>(imageData),
                   this->ping()->imageSize, strideBytes, this->ping()->nRanges);
  } else {
    _image =
        ImageData(imageData, this->ping()->imageSize, this->ping()->nRanges,
                  this->ping()->nBeams, SizeOfDataSize(this->ping()->dataSize));
  }
}

template <typename Ping_t>
const Ping_t *SimplePingResult<Ping_t>::ping() const {
  return reinterpret_cast<const Ping_t *>(this->buffer()->data());
}

template <typename Ping_t> bool SimplePingResult<Ping_t>::valid() const {
  if (this->buffer()->size() < sizeof(OculusMessageHeader))
    return false;
  if (this->buffer()->size() < this->packetSize())
    return false;

  if (!MessageHeader::valid()) {
    // LOG(WARNING) << "Header not valid";
    return false;
  }

  const size_t bearingsOffset = sizeof(Ping_t);
  const size_t bearingsBytes =
      static_cast<size_t>(ping()->nBeams) * sizeof(int16_t);
  if (bearingsOffset + bearingsBytes > this->buffer()->size()) {
    return false;
  }

  if (static_cast<size_t>(ping()->imageOffset) + ping()->imageSize >
      this->buffer()->size()) {
    return false;
  }

  int num_pixels = ping()->nRanges * ping()->nBeams;
  size_t expected_size = SizeOfDataSize(ping()->dataSize) * num_pixels;

  if (this->flags().getSendGain()) {
    expected_size += sizeof(uint32_t) * ping()->nRanges;
  }

  if (ping()->imageSize != expected_size) {
    // LOG(WARNING) << "ImageSize in header " << ping()->imageSize
    //              << " does not match expected data size of " <<
    //              expected_size;
    return false;
  }

  if (ping()->imageOffset <= sizeof(OculusSimplePingResult)) {
    return false;
  }
  return true;
}

template <typename Ping_t>
std::vector<std::string>
SimplePingResult<Ping_t>::dump(std::vector<std::string> &vec) const {
  // ostr << "--------------";
  SimpleFireMsg_t::dump(vec);

  vec.push_back(fmt::format("          Ping ID: {}", this->ping()->pingId));
  vec.push_back(fmt::format("           Status: {}", this->ping()->status));
  vec.push_back(
      fmt::format("  Ping start time: {}", this->ping()->pingStartTime));

  vec.push_back(fmt::format("        Frequency: {}", this->ping()->frequency));
  vec.push_back(
      fmt::format("      Temperature: {}", this->ping()->temperature));
  vec.push_back(fmt::format("         Pressure: {}", this->ping()->pressure));
  vec.push_back(
      fmt::format("Spd of sound used: {}", this->ping()->speedOfSoundUsed));
  vec.push_back(
      fmt::format(" Range resolution: {} m", this->ping()->rangeResolution));

  if (this->flags().getRangeAsMeters()) {
    vec.push_back(
        fmt::format("   Calc range: {} m",
                    this->ping()->rangeResolution * this->ping()->nRanges));
    //   ostr <<
    //              <<  << " m";
  } else {
    vec.push_back(
        fmt::format("   Pct range: {}",
                    this->ping()->rangeResolution * this->ping()->nRanges));
  }

  vec.push_back(fmt::format("       Num ranges: {}", this->ping()->nRanges));
  vec.push_back(fmt::format("        Num beams: {}", this->ping()->nBeams));

  vec.push_back(fmt::format("    Azimuth range: {} - {}", bearings().front(),
                            bearings().back()));

  vec.push_back(fmt::format("       Image size: {}", this->ping()->imageSize));
  vec.push_back(
      fmt::format("     Image offset: {}", this->ping()->imageOffset));
  vec.push_back(fmt::format("        Data size: {}",
                            DataSizeToString(this->ping()->dataSize)));
  vec.push_back(
      fmt::format("     Message size: {}", this->ping()->messageSize));

  // ostr << "--------------";

  return vec;
}

} // namespace liboculus
