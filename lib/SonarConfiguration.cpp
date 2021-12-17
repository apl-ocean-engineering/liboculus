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


#include "liboculus/SonarConfiguration.h"

#include <boost/asio.hpp>
#include <g3log/g3log.hpp>

namespace liboculus {

SonarConfiguration::SonarConfiguration() {
  memset(&_sfm, 0, sizeof(OculusSimpleFireMessage));

  // Fill in OculusMessageHeader _sfm.head
  _sfm.head.oculusId    = OCULUS_CHECK_ID;  // 0x4f53
  _sfm.head.srcDeviceId = 0;
  _sfm.head.dstDeviceId = 7892;             // Q(lindzey): Where does this come from?
  _sfm.head.msgId       = messageSimpleFire;
  _sfm.head.msgVersion  = 0;
  _sfm.head.payloadSize = sizeof(OculusSimpleFireMessage) - sizeof(OculusMessageHeader);

  _sfm.masterMode = OCULUS_HIGH_FREQ;
  _sfm.pingRate = pingRateNormal;
  _sfm.networkSpeed = 0xff;  // uint8_t; can reduce network speed for bad links
  _sfm.gammaCorrection = 127;  // uint8_t; for 127, gamma = 0.5

  // uint8_t flags;
  // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
  // bit 1: 0 = 8 bit data, 1 = 16 bit data
  // bit 2: 0 = won't send gain, 1 = send gain
  // bit 3: 0 = send full return message, 1 = send simple return message
  // bit 4: "gain assistance"?
  // bit 6: use 512 beams (vs 256): email from Blueprint said to set flags |= 0x40
  _sfm.flags = 0x59;  // 0x59 = 0b1011001. Send simple return msg; range in meters; 512 beams

  _sfm.range = 2;  // 2 m; can be percent or meters, flag controlled
  _sfm.gainPercent = 50;
  _sfm.speedOfSound = 0.0;  // m/s  0 to calculate SoS from salinity
  _sfm.salinity = 0.0;  // ppt; 0 for freshwater, 35 for seawater
}

// TODO(lindzey): I don't love that the driver knows how to
//   convert bools to flags. Better to do that here ...
void SonarConfiguration::setFlags(uint8_t flags) {
  _sfm.flags = flags;
}

void SonarConfiguration::setRange(double input) {
  // 40 meters is the max range for the 1200d model
  // may need to use a double instead of uint8_t (depends on flags)
  if (input <= 40 && input > 0) {
    _sfm.range = input;
  } else {
    LOG(WARNING) << "Requested invalid range: " << input;
  }
}

void SonarConfiguration::setGainPercent(double input) {
  if (input <= 100 && input > 0) {
    _sfm.gainPercent = input;
  } else {
    LOG(WARNING) << "Requested invalid gain: " << input;
  }
}

void SonarConfiguration::setGamma(int input) {
  if (input <= 255 && input > 0) {
    _sfm.gammaCorrection = input;
  } else {
    LOG(WARNING) << "Requested invalid gamma: " << input;
  }
}

void SonarConfiguration::setPingRate(PingRateType newRate) {
  _sfm.pingRate = newRate;
}

void SonarConfiguration::setFreqMode(OculusFreqMode input) {
  _sfm.masterMode = input;
}

std::vector<uint8_t> SonarConfiguration::serialize() const {
  std::vector<uint8_t> v;
  const auto ptr=reinterpret_cast<const char*>(&_sfm);
  v.insert(v.end(), ptr, ptr + sizeof(OculusSimpleFireMessage));
  return v;
}

}  // namespace liboculus
