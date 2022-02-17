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

// Thin OO wrapper around the OculusSimpleFireMessage.
//
// \todo This API is a little messy right now.  Probably *shouldn't* keep
// the shadow copy of the SimpleFireMessage{2}, instead
// just make it at time of serialization.
class SonarConfiguration {
 public:

  SonarConfiguration();

  template <typename FireMsg_t>
  std::vector<uint8_t> serialize() const;

  // Setter functions
  SonarConfiguration &setGamma(int input);
  SonarConfiguration &setPingRate(PingRateType newRate);
  SonarConfiguration &setGainPercent(double input);
  SonarConfiguration &setFlags(uint8_t flags);
  SonarConfiguration &setWaterTemperature(double degC);

  typedef enum {
    OCULUS_LOW_FREQ = 1,
    OCULUS_HIGH_FREQ = 2
  } OculusFreqMode;

  SonarConfiguration &setFreqMode(OculusFreqMode input);
  OculusFreqMode getFreqMode() const {
    if (_sfm.masterMode == 1)
      return OCULUS_LOW_FREQ;
    else if (_sfm.masterMode == 2)
      return OCULUS_HIGH_FREQ;
  }

  SonarConfiguration &setDataSize(DataSizeType sz);
  DataSizeType getDataSize() const         { return _dataSize; }

  SonarConfiguration &setRange(double input);

  SonarConfiguration &sendRangeAsMeters(bool v);
  SonarConfiguration &sendRangeAsMeters()  { return sendRangeAsMeters(true); }
  SonarConfiguration &sendRangeAsPercent() { return sendRangeAsMeters(false); }
  bool getSendRangeAsMeters() const        { return _sendRangeAsMeters; }


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


  bool getSendGain() const                { return _sendGain; }
  //bool getData16Bit() const               { return _16bitData; }
  bool getSimpleReturn() const            { return _simpleReturn; }
  bool getGainAssistance() const          { return _gainAssistance; }

  void dump() const;

 private:

  void updateFlags() const;

  mutable OculusSimpleFireMessage2 _sfm;

  bool _sendRangeAsMeters;
  float _rangeInMeters;

  bool _sendGain;
  bool _simpleReturn;
  bool _gainAssistance;
  bool _512beams;

  DataSizeType _dataSize;

};  // class SonarConfiguration

}  // namespace liboculus
