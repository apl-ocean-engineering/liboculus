// Copyright (c) 2017-2026 University of Washington
// Author: Aaron Marburg <amarburg@uw.edu>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of University of Washington nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <cassert>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "BearingData.h"
#include "DataTypes.h"
#include "ImageData.h"
#include "liboculus/thirdparty/Oculus/Oculus.h"

namespace liboculus {

using std::shared_ptr;
using std::vector;

/// This class serves two purposes.  It is both a "holder of a byte buffer"
//  as well as an overlay for accessing the MessageHeader struct represented
//  by the buffer.    Other packet types can inherit from this class
//  to represent large structs which start with a MessageHeader
class MessageHeader {
public:
  MessageHeader() = default;
  MessageHeader(const MessageHeader &) = default;

  explicit MessageHeader(const std::shared_ptr<ByteVector> &buffer)
      : _buffer(buffer) {
    assert(buffer->size() >= sizeof(OculusMessageHeader));
  }

  ~MessageHeader() {}

  // Convenience accessors
  OculusMessageType msgId() const {
    return static_cast<OculusMessageType>(hdr()->msgId);
  }
  uint16_t oculusId() const { return hdr()->oculusId; }
  uint16_t srcDeviceId() const { return hdr()->srcDeviceId; }
  uint16_t dstDeviceId() const { return hdr()->dstDeviceId; }
  uint16_t msgVersion() const { return hdr()->msgVersion; }
  uint32_t payloadSize() const { return hdr()->payloadSize; }
  uint32_t packetSize() const {
    return payloadSize() + sizeof(OculusMessageHeader);
  }

  virtual bool valid() const {
    return hdr()->oculusId == OCULUS_CHECK_ID; // 0x4f53
  }

  virtual std::vector<std::string> dump(std::vector<std::string> &vec) const {
    vec.push_back(fmt::format("   Oculus Id: {:#04x}", oculusId()));
    vec.push_back(
        fmt::format("      Msg id: {:#04x}", static_cast<uint16_t>(msgId())));
    vec.push_back(fmt::format(" Msg Version: {}", msgVersion()));
    vec.push_back(fmt::format("      Dst ID: {:#04x}", dstDeviceId()));
    vec.push_back(fmt::format("      Src ID: {:#04x}", srcDeviceId()));
    vec.push_back(fmt::format("Payload size: {} bytes", payloadSize()));

    return vec;
  }

  const OculusMessageHeader *hdr() const {
    return reinterpret_cast<const OculusMessageHeader *>(_buffer->data());
  }

  const std::shared_ptr<ByteVector> &buffer(void) const { return _buffer; }

protected:
  std::shared_ptr<ByteVector> _buffer;
}; // class MessageHeader

} // namespace liboculus
