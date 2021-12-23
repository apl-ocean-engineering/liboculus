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

#include "BearingData.h"
#include "DataTypes.h"
#include "ImageData.h"

#include "Oculus/Oculus.h"
#include "liboculus/MessageHeader.h"
#include <g3log/g3log.hpp>

namespace liboculus {

using std::shared_ptr;
using std::vector;

// A single OculusSimplePingResult (msg) is actually three nested structs:
//   OculusMessageHeader     (as msg.fireMessage.head)
//   OculusSimpleFireMessage (as msg.fireMessage)
//   then the rest of OculusSimplePingResult
class SimplePingResult : public MessageHeader {
 public:
  SimplePingResult() = default;
  SimplePingResult(const SimplePingResult &other) = default;

  explicit SimplePingResult(const std::shared_ptr<ByteVector> &buffer);

  ~SimplePingResult() {}

  const OculusSimpleFireMessage *fireMsg() const {
      return reinterpret_cast<const OculusSimpleFireMessage *>(_buffer->data());
  }

  const OculusSimplePingResult *ping() const  {
    return reinterpret_cast<const OculusSimplePingResult *>(_buffer->data());
  }

  const BearingData &bearings() const { return _bearings; }
  const ImageData &image() const      { return _image; }

  uint8_t dataSize() const { return SizeOfDataSize(ping()->dataSize); }

  bool valid() const override;
  void dump() const override;

 private:
  // Objects which create OOI overlays the _buffer for  easier interpretation
  BearingData _bearings;
  ImageData _image;

};  // class SimplePingResult

}  // namespace liboculus
