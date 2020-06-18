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

#include <fstream>
#include <iostream>

#include "Oculus/Oculus.h"

#include "g3log/g3log.hpp"

#include "liboculus/DataTypes.h"
#include "liboculus/SimplePingResult.h"
#include "liboculus/SonarPlayer.h"

namespace liboculus {

using namespace std;

#ifdef WITH_GPMF
shared_ptr<SonarPlayerBase> SonarPlayerBase::createGPMFSonarPlayer() {
  return shared_ptr<SonarPlayerBase>(new GPMFSonarPlayer());
}
#endif

/// Static function which automatically detects file type
shared_ptr<SonarPlayerBase> SonarPlayerBase::OpenFile(const string &filename) {
  std::ifstream f(filename);

  if (!f.is_open())
    return nullptr;

  char c;
  f.get(c);
  if (c == 0x44) {

    char d;
    f.get(d);

    if (d == 0x45) {
#ifdef WITH_GPMF
      LOG(INFO) << "I think this is an GPMF file.";
      return shared_ptr<SonarPlayerBase>(new GPMFSonarPlayer());
#else
      LOG(WARNING) << "I think this is a GPMF file, but GPMF support is not compiled in.";
      return nullptr;
#endif
    }

    LOG(INFO) << "I think this is an Oculus client file.";
    return shared_ptr<SonarPlayerBase>(new OculusSonarPlayer());
  } else if (c == 0x53) {
    LOG(INFO) << "I think this is an raw sonar data.";
    return shared_ptr<SonarPlayerBase>(new RawSonarPlayer());
  }

  return nullptr;
}

//--- SonarPlayerBase

SonarPlayerBase::SonarPlayerBase() { ; }

SonarPlayerBase::~SonarPlayerBase() { ; }

bool SonarPlayerBase::open(const std::string &filename) {
  _input.open(filename, ios_base::binary | ios_base::in);

  return _input.is_open();
}

//--- RawSonarPlayer --

RawSonarPlayer::RawSonarPlayer() : SonarPlayerBase() { ; }

RawSonarPlayer::~RawSonarPlayer() { ; }

bool RawSonarPlayer::nextPacket( MessageHeader &header ) {

  // Advance to the next header byte (actually LSB of header since we're
  // little-endian)
  while (_input.peek() != 0x53) {
    char c;
    _input.get(c);
    if (_input.eof())
      return false;
  }

  // Read header
  header.reset();
  _input.read( (char *)header.ptr(), sizeof(OculusMessageHeader));

  if (!header.valid()) {
    LOG(WARNING) << "Incoming header invalid";
    return false;
  }

  header.expandForPayload();
  _input.read( (char *)header.payloadPtr(), header.alignedPayloadSize() );

  return true;
}

bool RawSonarPlayer::nextPing( SimplePingResult &ping ) {

  MessageHeader header;
  while (bool(nextPacket(header))) {

    if (header.msgId() == messageSimplePingResult) {
      ping = SimplePingResult(header);
      return true;
    } else {
      LOG(DEBUG) << "Skipping message of type " << MessageTypeToString(header.msgId());
    }
  }

  return false;
}

//--- OculusSonarPlayer --

OculusSonarPlayer::OculusSonarPlayer() : SonarPlayerBase() { ; }

OculusSonarPlayer::~OculusSonarPlayer() { ; }

bool OculusSonarPlayer::nextPing( SimplePingResult &ping) {
  //
  // Ended up not being able to implement.  Oculus client records
  // messagePingResult, which we don't have the format for ...
  //

  return false;
}

#ifdef WITH_GPMF

//--- GPMFSonarPlayer --

GPMFSonarPlayer::GPMFSonarPlayer()
    : SonarPlayerBase(), _stream(), _valid(false), _buffer() {
  ;
}

GPMFSonarPlayer::~GPMFSonarPlayer() { ; }

bool GPMFSonarPlayer::open(const std::string &filename) {
  {
    bool retval = SonarPlayerBase::open(filename);
    if (!retval)
      return retval;
  }

  _input.seekg(0, std::ios::end);
  const size_t sz = _input.tellg();
  _buffer.resize(sz, '\0');
  _input.seekg(0, std::ios::beg);

  _input.read(&_buffer[0], sz);

  // _buffer.assign((std::istreambuf_iterator<char>(_input)),
  //             std::istreambuf_iterator<char>());

  LOG(DEBUG) << "Loading " << _buffer.size() << " bytes";

  GPMF_Init(&_stream, (unsigned int *)_buffer.c_str(), (_buffer.size()));

  // {
  //   auto retval = GPMF_Validate(&_stream, GPMF_RECURSE_LEVELS);
  //   if( retval != GPMF_OK ) {
  //     LOG(WARNING) << "GPMF structure is not valid; err = " << retval;
  //     return false;
  //   }
  // }

  {
    auto retval =
        GPMF_FindNext(&_stream, STR2FOURCC("OCUS"), GPMF_RECURSE_LEVELS);

    if (retval != GPMF_OK) {
      LOG(INFO) << "Unable to find Oculus sonar data in GPMF file (err "
                << retval << ")";
      return false;
    }
  }

  return true;
}

bool GPMFSonarPlayer::setStream(GPMF_stream *stream) {
  _stream = *stream;
  // GPMF_Init(&_stream, (unsigned int *)_buffer.c_str(), (_buffer.size()));
  //
  // {
  //   auto retval =
  //       GPMF_FindNext(&_stream, STR2FOURCC("OCUS"), GPMF_RECURSE_LEVELS);
  //
  //   if (retval != GPMF_OK) {
  //     LOG(INFO) << "Unable to find Oculus sonar data in GPMF file (err "
  //               << retval << ")";
  //     return false;
  //   }
  // }
  return true;
}

bool GPMFSonarPlayer::eof() {
  // How to handle this?
  auto retval = GPMF_Validate(&_stream, GPMF_RECURSE_LEVELS);
  return retval == GPMF_ERROR_BUFFER_END;
}

void GPMFSonarPlayer::rewind() {
  if (_valid) {
    GPMF_ResetState(&_stream);
  }
}

void GPMFSonarPlayer::close() { _valid = false; }

void GPMFSonarPlayer::dumpGPMF() {
  auto key = GPMF_Key(&_stream);
  LOG(INFO) << "Current key \"" << char((key >> 0) & 0xFF)
            << char((key >> 8) & 0xFF) << char((key >> 16) & 0xFF)
            << char((key >> 24) & 0xFF) << "\" (" << std::hex << key << ")";
  LOG(INFO) << "Current type " << GPMF_Type(&_stream);
  LOG(INFO) << "Current device ID " << std::hex << GPMF_DeviceID(&_stream);

  char deviceName[80];
  GPMF_DeviceName(&_stream, deviceName, 79);
  LOG(INFO) << "Current device name " << deviceName;

  LOG(INFO) << "Current struct size " << GPMF_StructSize(&_stream);
  LOG(INFO) << "Current repeat size " << GPMF_Repeat(&_stream);
  LOG(INFO) << "Current payload sample count "
            << GPMF_PayloadSampleCount(&_stream);
  LOG(INFO) << "Current elements in struct " << GPMF_ElementsInStruct(&_stream);
  LOG(INFO) << "Current raw data size " << GPMF_RawDataSize(&_stream);
}

bool GPMFSonarPlayer::nextPing( SimplePingResult &ping ) {
  //
  // Ended up not needing to implement.  Oculus client records
  // messagePingResult, which we don't have the format for ...
  auto key = GPMF_Key(&_stream);
  if (key != STR2FOURCC("OCUS"))
    return false;

  MessageHeader header( (char *)GPMF_RawData(&_stream), GPMF_RawDataSize(&_stream) );
  // char *data = (char *)GPMF_RawData(&_stream);
  // CHECK(data != nullptr);

  if (!header.valid()) {
    LOG(INFO) << "Invalid header";
    return false;
  }

  auto retval = GPMF_FindNext(&_stream, STR2FOURCC("OCUS"), GPMF_RECURSE_LEVELS);
  if (retval != GPMF_OK) {
    _valid = false;
  }

  ping = SimplePingResult( header );
  return true;
}

#endif

} // namespace liboculus
