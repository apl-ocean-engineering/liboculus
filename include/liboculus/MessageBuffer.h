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
  MessageBuffer( int reserve = sizeof(OculusMessageHeader) )
    : _buf( reserve, 0)
  { ; }

  MessageBuffer(const char *data, size_t len)
    : _buf(len)
  { memcpy(_buf.data(), data, len); }

  MessageBuffer(const std::vector<char> &vec)
    : _buf(vec)
  { ; }

  ~MessageBuffer()
  { ; }

  char *ptr() { return _buf.data(); }
  const char *ptr() const { return _buf.data(); }

//  const char *headerPtr() { return ptr(); }

  char *payloadPtr() {
    return _buf.data() + sizeof(OculusMessageHeader);
  }

  // const char *payloadPtr() const  {
  //   return _buf.data() + sizeof(OculusMessageHeader);
  // }

  unsigned int size() const {
    return _buf.size();
  }

  unsigned int payloadSize() const {
    return _buf.size() - sizeof(OculusMessageHeader);
  }

  bool expandForPayload() {
    OculusMessageHeader *hdr =
        reinterpret_cast<OculusMessageHeader *>(_buf.data());
    if (hdr->oculusId != 0x4f53)
      return false;

    // alloc a 4-byte aligned buffer
    const size_t dataSize = sizeof(OculusMessageHeader) + hdr->payloadSize;
    const size_t alignSize = ((dataSize + 3) >> 2) << 2;
    _buf.resize(alignSize);

    return true;
  }

protected:
  std::vector<char> _buf;
};

}
