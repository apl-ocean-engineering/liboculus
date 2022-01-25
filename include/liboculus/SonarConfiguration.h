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

#include <vector>
#include <boost/asio.hpp>

#include "Oculus/Oculus.h"

namespace liboculus {


class OculusSimpleFireFlags {
  // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
  // bit 1: 0 = 8 bit data, 1 = 16 bit data
  // bit 2: 0 = wont send gain, 1 = send gain
  // bit 3: 0 = send full return message, 1 = send simple return message
  // bit 4: "gain assistance"?
  // bit 6: use 512 beams (vs 256): email from Blueprint said to set flags |= 0x40
static const uint8_t RangeAsMeters = (0x01) << 0;
static const uint8_t Data16Bits    = (0x01) << 1;
static const uint8_t DoSendGain    = (0x01) << 2;
static const uint8_t SimpleReturn  = (0x01) << 3;
static const uint8_t GainAssistance = (0x01) << 4;
static const uint8_t Do512Beams    = (0x01) << 6;

 public:
  OculusSimpleFireFlags();
  OculusSimpleFireFlags(uint8_t flags);

  // Serialize values
  uint8_t operator()() const;

  OculusSimpleFireFlags &setRangeAsMeters(bool v);
  OculusSimpleFireFlags &setData16Bit(bool v);
  OculusSimpleFireFlags &setSendGain(bool v);
  OculusSimpleFireFlags &setSimpleReturn(bool v);
  OculusSimpleFireFlags &setGainAssistance(bool v);
  OculusSimpleFireFlags &set512Beams(bool v);

  bool getRangeAsMeters() const           { return _rangeAsMeters; }
  bool getSendGain() const                { return _sendGain; }
  bool getData16Bit() const               { return _16bitData; }
  bool getSimpleReturn() const            { return _simpleReturn; }
  bool getGainAssistance() const          { return _gainAssistance; }
  bool get512Beams() const                { return _512beams;}

  // Convenient setters
  OculusSimpleFireFlags &rangeAsMeters()  { return setRangeAsMeters(true); }
  OculusSimpleFireFlags &rangeAsPercent() { return setRangeAsMeters(false); }
  OculusSimpleFireFlags &data8Bit()       { return setData16Bit(false); }
  OculusSimpleFireFlags &data16Bit()      { return setData16Bit(true); }
  OculusSimpleFireFlags &sendGain()       { return setSendGain(true); }
  OculusSimpleFireFlags &dontSendGain()   { return setSendGain(false); }
  OculusSimpleFireFlags &use256Beams()    { return set512Beams(false); }
  OculusSimpleFireFlags &use512Beams()    { return set512Beams(true); }
  OculusSimpleFireFlags &gainAssistance() { return setGainAssistance(true); }
  OculusSimpleFireFlags &noGainAssistance() { return setGainAssistance(false); }

 protected:

  OculusSimpleFireFlags &setBit(uint8_t bit, bool v);

  bool _rangeAsMeters;
  bool _16bitData;
  bool _sendGain;
  bool _simpleReturn;
  bool _gainAssistance;
  bool _512beams;
};


// Thin OO wrapper around the OculusSimpleFireMessage.
class SonarConfiguration {
 public:
  SonarConfiguration();

  std::vector<uint8_t> serializeFireMsg() const;
  std::vector<uint8_t> serializeFireMsg2() const;

  // Setter functions
  void setGamma(int input);
  void setPingRate(PingRateType newRate);
  void setGainPercent(double input);
  void setRange(double input);
  void setFlags(uint8_t flags);
  void setWaterTemperature(double degC);

  typedef enum {
    OCULUS_LOW_FREQ = 1,
    OCULUS_HIGH_FREQ = 2
  } OculusFreqMode;

  void setFreqMode(OculusFreqMode input);

  OculusSimpleFireFlags &flags() { return _flags; }
  const OculusSimpleFireFlags &flags() const { return _flags; }


  void dump() const;

 private:
  OculusSimpleFireFlags _flags;
  mutable OculusSimpleFireMessage _sfm;
};  // class SonarConfiguration

}  // namespace liboculus
