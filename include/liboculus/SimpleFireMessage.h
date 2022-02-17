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

#include "Oculus/Oculus.h"
#include "liboculus/MessageHeader.h"
#include "liboculus/SonarConfiguration.h"
#include "liboculus/Constants.h"

namespace liboculus {

using std::shared_ptr;
using std::vector;



// A simple parser of flag bytes
class OculusSimpleFireFlags {
 public:
    OculusSimpleFireFlags() = delete;
    OculusSimpleFireFlags( const uint8_t flags )
    : _data(flags) {;}

    bool getRangeAsMeters() const  { return _data & FlagBits::RangeAsMeters; }
    bool getSendGain() const       { return _data & FlagBits::DoSendGain; }
    bool getData16Bit() const      { return _data & FlagBits::Data16Bits; }
    bool getSimpleReturn() const   { return _data & FlagBits::SimpleReturn; }
    bool getGainAssistance() const { return _data & FlagBits::GainAssistance; }
    bool get512Beams() const       { return _data & FlagBits::Do512Beams; }

 private:
    uint8_t _data;
};

template <typename FireMsgT>
class SimpleFireMessage : public MessageHeader {
 public:
  SimpleFireMessage() = default;
  SimpleFireMessage(const SimpleFireMessage &other) = default;

  explicit SimpleFireMessage(const std::shared_ptr<ByteVector> &buffer);

  ~SimpleFireMessage() {}

  const FireMsgT *fireMsg() const {
      return reinterpret_cast<const FireMsgT *>(_buffer->data());
  }

  const OculusSimpleFireFlags &flags() const {
    return _flags;
  }

  void dump() const override;

  float range() const;

 protected:
  OculusSimpleFireFlags _flags;
};  // class SimpleFireMessage


template<typename FireMsgT>
SimpleFireMessage<FireMsgT>::SimpleFireMessage(const std::shared_ptr<ByteVector> &buffer)
  : MessageHeader(buffer),
  _flags(this->fireMsg()->flags) {
  assert(buffer->size() >= sizeof(FireMsgT));
}

template<typename FireMsgT>
void SimpleFireMessage<FireMsgT>::dump() const {
  MessageHeader::dump();

  LOG(DEBUG) << "        Mode: " << FreqModeToString(this->fireMsg()->masterMode);

  const int pingRate = PingRateToHz(this->fireMsg()->pingRate);
  if (pingRate >= 0 ) {
    LOG(DEBUG) << "   Ping rate: " << pingRate;
  } else {
    LOG(DEBUG) << "   Ping rate: (unknown) " << static_cast<int>(this->fireMsg()->pingRate);
  }

  LOG(DEBUG) << "Flags: Send gain: " << (this->flags().getSendGain() ? "Yes" : "No");
  LOG(DEBUG) << "Flags:  Range is: " << (this->flags().getRangeAsMeters() ? "Meters" : "Percent");
  LOG(DEBUG) << "       Range: " << range();
  LOG(DEBUG) << "Speed of sound: " << this->fireMsg()->speedOfSound;
}

}  // namespace liboculus
