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

struct FlagBits {
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
};

// This class used to be both a way to store state and
// deserializer.  But it got awkward, so now it's 
// *only* a parser of flag bytes (does no allow setting)
// configuration is stored outside of the bitfield in
// SonarConfiguration, and the flag byte is made just-in-time
class OculusSimpleFireFlags {
 public:
    OculusSimpleFireFlags() = delete;
    OculusSimpleFireFlags( const uint8_t flags )
    : _data(flags) {;}

    bool getRangeAsMeters() const           { return _data & FlagBits::RangeAsMeters; }
    bool getSendGain() const                { return _data & FlagBits::DoSendGain; }
    bool getData16Bit() const               { return _data & FlagBits::Data16Bits; }
    bool getSimpleReturn() const            { return _data & FlagBits::SimpleReturn; }
    bool getGainAssistance() const          { return _data & FlagBits::GainAssistance; }
    bool get512Beams() const                { return _data & FlagBits::Do512Beams;}

 private:
    uint8_t _data;
};

// Thin OO wrapper around the OculusSimpleFireMessage.
class SonarConfiguration {
 public:

  SonarConfiguration();

  template <typename FireMsg_t>
  std::vector<uint8_t> serialize() const;

  // Setter functions
  SonarConfiguration &setGamma(int input);
  SonarConfiguration &setPingRate(PingRateType newRate);
  SonarConfiguration &setGainPercent(double input);
  SonarConfiguration &setRange(double input);
  SonarConfiguration &setFlags(uint8_t flags);
  SonarConfiguration &setWaterTemperature(double degC);

  typedef enum {
    OCULUS_LOW_FREQ = 1,
    OCULUS_HIGH_FREQ = 2
  } OculusFreqMode;

  SonarConfiguration &setFreqMode(OculusFreqMode input);

  SonarConfiguration &setDataSize(DataSizeType sz);
  DataSizeType getDataSize() const { return _dataSize; }

  SonarConfiguration &setRangeAsMeters(bool v);
  SonarConfiguration &rangeAsMeters()  { return setRangeAsMeters(true); }
  SonarConfiguration &rangeAsPercent() { return setRangeAsMeters(false); }

  SonarConfiguration &setSendGain(bool v);
  SonarConfiguration &sendGain()       { return setSendGain(true); }
  SonarConfiguration &dontSendGain()   { return setSendGain(false); }

  SonarConfiguration &setSimpleReturn(bool v);

  SonarConfiguration &setGainAssistance(bool v);
  SonarConfiguration &gainAssistance() { return setGainAssistance(true); }
  SonarConfiguration &noGainAssistance() { return setGainAssistance(false); }

  SonarConfiguration &set512Beams(bool v);
  SonarConfiguration &use256Beams()    { return set512Beams(false); }
  SonarConfiguration &use512Beams()    { return set512Beams(true); }
  bool get512Beams() const                { return _512beams;}


  bool getRangeAsMeters() const           { return _rangeAsMeters; }
  bool getSendGain() const                { return _sendGain; }
  //bool getData16Bit() const               { return _16bitData; }
  bool getSimpleReturn() const            { return _simpleReturn; }
  bool getGainAssistance() const          { return _gainAssistance; }

  void dump() const;

 private:

  void updateFlags() const;

  mutable OculusSimpleFireMessage2 _sfm;

  bool _rangeAsMeters;
  bool _sendGain;
  bool _simpleReturn;
  bool _gainAssistance;
  bool _512beams;

  DataSizeType _dataSize;

};  // class SonarConfiguration

}  // namespace liboculus
