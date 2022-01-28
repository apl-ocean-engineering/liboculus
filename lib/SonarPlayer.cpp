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

#include <fstream>
#include <iostream>

#include "Oculus/Oculus.h"

#include "g3log/g3log.hpp"

#include "liboculus/DataTypes.h"
#include "liboculus/MessageHeader.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/SonarPlayer.h"
#include "liboculus/Constants.h"

namespace liboculus {

using std::shared_ptr;
using std::ios_base;

using liboculus::MessageHeader;
using liboculus::SimplePingResult;

/// Static function which automatically detects file type
shared_ptr<SonarPlayerBase> SonarPlayerBase::OpenFile(const std::string &filename) {
  std::ifstream f(filename);

  if (!f.is_open())
    return nullptr;

  char c;
  f.get(c);
  if (c == 0x44) {
    char d;
    f.get(d);

    if (d == 0x45) {
      LOG(INFO) << "I think this is an GPMF file, unfortunately I cannot parse GPMF";
      return nullptr;
    }

  } else if (c == 0x53) {
    LOG(INFO) << "I think this is an raw sonar data.";
    return shared_ptr<SonarPlayerBase>(new RawSonarPlayer());
  }

  LOG(INFO) << "Unable to figure out what file this is...";
  return nullptr;
}

//--- SonarPlayerBase

bool SonarPlayerBase::open(const std::string &filename) {
  _input.open(filename, ios_base::binary | ios_base::in);
  return _input.is_open();
}

//--- RawSonarPlayer --

bool RawSonarPlayer::nextPing() {
  unsigned int skipped_bytes = 0;
  while (_input.peek() != PacketHeaderLSB) {
    char c;
    _input.get(c);
    skipped_bytes++;
    if (_input.eof()) {
      LOG(DEBUG) << "No packets before the end of the file";
      return false;
    }
  }

  LOG_IF(INFO, skipped_bytes > 0) << "Skipped " << skipped_bytes << " before reading start of header";

  std::shared_ptr<ByteVector> buffer = std::make_shared<ByteVector>(sizeof(MessageHeader));
  _input.get(reinterpret_cast<char *>(buffer->data()), sizeof(MessageHeader));

  MessageHeader header(buffer);
  if (!header.valid()) return false;

  // Read the rest of the data
  buffer->resize(header.packetSize());
  _input.get(reinterpret_cast<char *>(buffer->data()[sizeof(MessageHeader)]),header.payloadSize());

  if (header.msgId() == messageSimplePingResult) {
    if (header.msgVersion() == 2) {
      callback(SimplePingResultV2(buffer));
    } else {
      callback(SimplePingResultV1(buffer));
    }    
  }

  return true; 
}


}  // namespace liboculus
