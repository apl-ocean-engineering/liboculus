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


#include <iomanip>

#include "liboculus/SonarConfiguration.h"
#include "liboculus/DataTypes.h"
#include "liboculus/Constants.h"

#include <boost/asio.hpp>
#include <g3log/g3log.hpp>

namespace liboculus {

SonarConfiguration::SonarConfiguration()
: _sendRangeAsMeters(true),
  _rangeInMeters(5),
  _sendGain(true),
  _simpleReturn(true),
  _gainAssistance(true),
  _512beams(true),
  _dataSize(dataSize8Bit) {
  memset(&_sfm, 0, sizeof(OculusSimpleFireMessage));

  // Fill in OculusMessageHeader _sfm.head
  _sfm.head.oculusId    = OCULUS_CHECK_ID;  // 0x4f53
  _sfm.head.srcDeviceId = 0;
  _sfm.head.dstDeviceId = 0;                // n.b. ignored by device
  _sfm.head.msgId       = messageSimpleFire;
  _sfm.head.msgVersion  = 2;
  _sfm.head.payloadSize = sizeof(OculusSimpleFireMessage2) - sizeof(OculusMessageHeader);

  _sfm.masterMode = OCULUS_HIGH_FREQ;
  _sfm.pingRate = pingRateNormal;
  _sfm.networkSpeed = 0xff;    // uint8_t; can reduce network speed for bad links
  _sfm.gammaCorrection = 127;  // uint8_t; for 127, gamma = 0.5

  updateFlags();   // Set to defaults

  _sfm.rangePercent = 2;  // 2 m; can be percent or meters, flag controlled
  _sfm.gainPercent = 50;
  _sfm.speedOfSound = 0.0;  // m/s  0 to calculate SoS from salinity
  _sfm.salinity = 0.0;  // ppt; 0 for freshwater, 35 for seawater
}

SonarConfiguration &SonarConfiguration::setRange(double input) {
  // 40 meters is the max range for the 1200d model
  // may need to use a double instead of uint8_t (depends on flags)
  if (input <= 40 && input > 0) {
    _rangeInMeters = input;
  } else {
    LOG(WARNING) << "Requested invalid range: " << input;
  }
  return *this;
}

SonarConfiguration &SonarConfiguration::setGainPercent(double input) {
  if (input <= 100 && input > 0) {
    _sfm.gainPercent = input;
  } else {
    LOG(WARNING) << "Requested invalid gain: " << input;
  }
  return *this;
}

SonarConfiguration &SonarConfiguration::setGamma(int input) {
  if (input <= 255 && input > 0) {
    _sfm.gammaCorrection = input;
  } else {
    LOG(WARNING) << "Requested invalid gamma: " << input;
  }
  return *this;
}

SonarConfiguration &SonarConfiguration::setPingRate(PingRateType newRate) {
  _sfm.pingRate = newRate;
  return *this;
}

SonarConfiguration &SonarConfiguration::setFreqMode(OculusFreqMode input) {
  _sfm.masterMode = input;
  return *this;
}

SonarConfiguration &SonarConfiguration::setDataSize(DataSizeType sz) {
  _dataSize = sz;
  return *this;
}

SonarConfiguration &SonarConfiguration::sendRangeAsMeters(bool v) {
  _sendRangeAsMeters = v;
  return *this;
}

SonarConfiguration &SonarConfiguration::setSendGain(bool v) {
  _sendGain = v;
  return *this;
}

SonarConfiguration &SonarConfiguration::setSimpleReturn(bool v) {
  _simpleReturn = v;
  return *this;
}

SonarConfiguration &SonarConfiguration::setGainAssistance(bool v) {
  _gainAssistance = v;
  return *this;
}

SonarConfiguration &SonarConfiguration::set512Beams(bool v) {
  _512beams = v;
  return *this;
}


//== Serialization functions

template <>
std::vector<uint8_t> SonarConfiguration::serialize<OculusSimpleFireMessage2>() const {
  updateFlags();

  // Corner case.  If in 32bit mode, range is always a percentage of max range
  // if ((_dataSize == dataSize32Bit) || (!_sendRangeAsMeters)) {
  //   const float maxRange = ((getFreqMode()==OCULUS_LOW_FREQ) ? Oculus_1200MHz::MaxRange : Oculus_2100MHz::MaxRange);

  //   _sfm.rangePercent = std::min(_rangeInMeters/maxRange * 100.0,100.0);


  //   LOG(INFO) << "In 32bit mode, setting range to " << _rangeInMeters << " which is " << _sfm.rangePercent << " percent";
  // } else {
    _sfm.rangePercent = _rangeInMeters;
  //}

  std::vector<uint8_t> v;
  const auto ptr = reinterpret_cast<const char*>(&_sfm);
  v.insert(v.end(), ptr, ptr + sizeof(OculusSimpleFireMessage2));

  return v;
}

template <>
std::vector<uint8_t> SonarConfiguration::serialize<OculusSimpleFireMessage>() const {
  updateFlags();

  // As of right now, since OculusSimpleFireMessage and OculusSimpleFireMessage2
  // have the same fields in the same order (but different length)
  // Just memcpy and revise and necessary fields

  OculusSimpleFireMessage sfm;
  memcpy(reinterpret_cast<void *>(&sfm), 
         reinterpret_cast<const void *>(&_sfm), 
         sizeof(OculusSimpleFireMessage));

  // Rewrite any fields which are different between SimpleFireMessage and SimpleFireMessage2
  sfm.head.msgVersion = 1;
  sfm.head.payloadSize = sizeof(OculusSimpleFireMessage) - sizeof(OculusMessageHeader);

  std::vector<uint8_t> v;
  const auto ptr = reinterpret_cast<const char*>(&sfm);
  v.insert(v.end(), ptr, ptr + sizeof(OculusSimpleFireMessage));
  return v;
}

void SonarConfiguration::updateFlags() const {
  _sfm.extFlags = 0;
  if (_dataSize == dataSize32Bit) 
    _sfm.extFlags |= 0x00000200;

  _sfm.flags = (_rangeInMeters  ? FlagBits::RangeAsMeters : 0 ) |
         (((_dataSize == dataSize16Bit) || (_dataSize == dataSize32Bit)) ? FlagBits::Data16Bits : 0) |
         (_sendGain       ? FlagBits::DoSendGain : 0) |
         (_simpleReturn   ? FlagBits::SimpleReturn : 0) |
         (_gainAssistance ? FlagBits::GainAssistance : 0) |
         (_512beams       ? FlagBits::Do512Beams : 0);
}


void SonarConfiguration::dump() const {
    updateFlags();

    LOG(INFO) << "\n             Flags 0x"
            << std::hex << std::setw(2) << std::setfill('0')
            << static_cast<unsigned int>(_sfm.flags)
            << std::setw(8)
            << "\n            Ext flags 0x"
            << std::setw(8) << static_cast<uint32_t>(_sfm.extFlags)
            << std::dec << std::setw(0)
            << "\n  send range is meters " << getSendRangeAsMeters()
            << "\n       data size       " << DataSizeToString(getDataSize())
            << "\n       send gain       " << getSendGain()
            << "\n       simple return   " << getSimpleReturn()
            << "\n       gain assistance " << getGainAssistance()
            << "\n       use 512 beams   " << get512Beams();
}




}  // namespace liboculus
