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

#include "liboculus/SonarPlayer.h"

#include <fstream>

#include "liboculus/Constants.h"
#include "liboculus/DataTypes.h"
#include "liboculus/Logger.h"
#include "liboculus/MessageHeader.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/thirdparty/Oculus/Oculus.h"

namespace liboculus {

using std::ios_base;
using std::shared_ptr;

using liboculus::MessageHeader;
using liboculus::SimplePingResult;

/// Static function which automatically detects file type
shared_ptr<SonarPlayerBase>
SonarPlayerBase::OpenFile(const std::string &filename) {
  std::ifstream f(filename);

  if (!f.is_open()) {
    oclog::warn("OpenFile: unable to open {}", filename);
    return nullptr;
  }

  char c;
  f.get(c);
  if (c == 0x44) {
    char d;
    f.get(d);

    if (d == 0x45) {
      // LOG(INFO)
      //     << "I think this is an GPMF file, unfortunately I cannot parse
      //     GPMF";
      return nullptr;
    }

  } else if (c == 0x53) {
    oclog::debug("OpenFile: raw sonar data detected");
    // LOG(INFO) << "I think this is an raw sonar data.";
    return shared_ptr<SonarPlayerBase>(new RawSonarPlayer());
  }

  // LOG(INFO) << "Unable to figure out what file this is...";
  return nullptr;
}

//--- SonarPlayerBase

bool SonarPlayerBase::open(const std::string &filename) {
  _input.open(filename, ios_base::binary | ios_base::in);
  if (!_input.is_open()) {
    oclog::error("SonarPlayerBase: failed to open {}", filename);
  }
  return _input.is_open();
}

//--- RawSonarPlayer --

bool RawSonarPlayer::nextPing() {
  if (!_input.is_open()) {
    oclog::warn("RawSonarPlayer: input not open");
    return false;
  }

  unsigned int skipped_bytes = 0;
  while (_input.peek() != PacketHeaderLSB) {
    char c;
    _input.get(c);
    skipped_bytes++;
    if (_input.eof()) {
      // LOG(DEBUG) << "No packets before the end of the file";
      return false;
    }
  }

  if (skipped_bytes > 0) {
    oclog::debug("RawSonarPlayer: skipped {} bytes", skipped_bytes);
  }

  std::shared_ptr<ByteVector> buffer =
      std::make_shared<ByteVector>(sizeof(OculusMessageHeader));
  _input.read(reinterpret_cast<char *>(buffer->data()),
              sizeof(OculusMessageHeader));
  if (_input.gcount() !=
      static_cast<std::streamsize>(sizeof(OculusMessageHeader))) {
    oclog::warn("RawSonarPlayer: short read on header (got {})",
                _input.gcount());
    return false;
  }

  MessageHeader header(buffer);
  if (!header.valid()) {
    oclog::warn("RawSonarPlayer: invalid header oculusId={:#04x}",
                header.oculusId());
    return false;
  }

  oclog::debug("RawSonarPlayer: msgId={:#04x} ver={} payload={} packet={}",
               static_cast<uint16_t>(header.msgId()), header.msgVersion(),
               header.payloadSize(), header.packetSize());

  // LOG(DEBUG) << "Reading " << header.payloadSize() << " additional bytes";

  // Read the rest of the data
  buffer->resize(header.packetSize());
  _input.read(
      reinterpret_cast<char *>(buffer->data() + sizeof(OculusMessageHeader)),
      header.payloadSize());
  if (_input.gcount() != static_cast<std::streamsize>(header.payloadSize())) {
    oclog::warn("RawSonarPlayer: short read on payload (wanted {}, got {})",
                header.payloadSize(), _input.gcount());
    return false;
  }

  if (header.msgId() == messageSimplePingResult) {
    if (header.msgVersion() == 2) {
      SimplePingResultV2 ping(buffer);
      const bool ok = ping.valid();
      oclog::debug("RawSonarPlayer: ping v2 valid={}", ok);
      if (!ok) {
        oclog::warn(
            "RawSonarPlayer: invalid ping v2 (offset={}, size={}, beams={}, "
            "ranges={}, dataSize={})",
            ping.ping()->imageOffset, ping.ping()->imageSize,
            ping.ping()->nBeams, ping.ping()->nRanges,
            static_cast<int>(ping.ping()->dataSize));
        return false;
      }
      oclog::debug("RawSonarPlayer: invoking v2 callback");
      callback(ping);
    } else {
      SimplePingResultV1 ping(buffer);
      const bool ok = ping.valid();
      oclog::debug("RawSonarPlayer: ping v1 valid={}", ok);
      if (!ok) {
        oclog::warn(
            "RawSonarPlayer: invalid ping v1 (offset={}, size={}, beams={}, "
            "ranges={}, dataSize={})",
            ping.ping()->imageOffset, ping.ping()->imageSize,
            ping.ping()->nBeams, ping.ping()->nRanges,
            static_cast<int>(ping.ping()->dataSize));
        return false;
      }
      oclog::debug("RawSonarPlayer: invoking v1 callback");
      callback(ping);
    }
  }

  return true;
}

} // namespace liboculus
