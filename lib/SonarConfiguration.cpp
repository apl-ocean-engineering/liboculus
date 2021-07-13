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

namespace liboculus {

SonarConfiguration::SonarConfiguration()
    : postpone_callback_(false),
      _callback() {
  memset(&_sfm, 0, sizeof(OculusSimpleFireMessage));

  // TODO(lindzey): replace (here and everywhere) with OCULUS_CHECK_ID
  _sfm.head.oculusId    = OCULUS_CHECK_ID;  // 0x4f53
  _sfm.head.srcDeviceId = 0;
  _sfm.head.dstDeviceId = 7892;  // Q(lindzey): Where does this come from?
  _sfm.head.msgId       = messageSimpleFire;
  _sfm.head.msgVersion  = 0;
  _sfm.head.payloadSize = sizeof(OculusSimpleFireMessage) - sizeof(OculusMessageHeader);

  // OculusMessageHeader head;     // The standard message header
  //
  // uint8_t masterMode;
  // * mode 0 is flexi mode, needs full fire message (not available for third party developers)
  // * mode 1 - Low Frequency Mode (wide aperture, navigation)
  // * mode 2 - High Frequency Mode (narrow aperture, target identification)
  //
  // PingRateType pingRate;        // Sets the maximum ping rate.
  // uint8_t networkSpeed;         // Used to reduce the network comms speed (useful for high latency shared links)
  // uint8_t gammaCorrection;      // 0 and 0xff = gamma correction = 1.0
  //                               // Set to 127 for gamma correction = 0.5

  // double range;                 // The range demand in percent or m depending on flags
  // double gainPercent;           // The gain demand
  // double speedOfSound;          // ms-1, if set to zero then internal calc will apply using salinity
  // double salinity;              // ppt, set to zero if we are in fresh water

  _sfm.masterMode = OCULUS_HIGH_FREQ;

  _sfm.networkSpeed = 0xff;

  // Initial values
  _sfm.gammaCorrection = 127;  //gamma;
  _sfm.pingRate        = pingRateNormal;
  _sfm.range           = 2;  // Meters
  _sfm.gainPercent     = 50;  // gain;

  // uint8_t flags;
  // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
  // bit 1: 0 = 8 bit data, 1 = 16 bit data
  // bit 2: 0 = wont send gain, 1 = send gain
  // bit 3: 0 = send full return message, 1 = send simple return message
  // bit 4: "gain assistance"?

  _sfm.flags = 0x19;  // 0x19 = 0b11001. Send simple return msg; range in meters

  _sfm.speedOfSound    = 0.0;  // m/s  0 for automatic calculation speedOfSound;
  _sfm.salinity        = 0.0;  // ppt; freshwater salinity;
}

void SonarConfiguration::postponeCallback(void) {
  postpone_callback_ = true;
}

void SonarConfiguration::enableCallback(void) {
  postpone_callback_ = false;
  sendCallback();
}

void SonarConfiguration::sendCallback() {
  if (_callback && !postpone_callback_) {
     _callback(*this);
  }
}

// need to integrate flags into dyanmic reconfig
void SonarConfiguration::setFlags(uint8_t flags) {
  _sfm.flags = flags;
  sendCallback();
}

void SonarConfiguration::setRange(double input) {
  // 40 meters is the max range for the 1200d model
  // may need to use a double instead of uint8_t (depends on flags)
  if (input <= 40 && input > 0) {
    _sfm.range = input;
  }
  sendCallback();
}

void SonarConfiguration::setGainPercent(double input) {
  if (input <= 100 && input > 0) {
    _sfm.gainPercent = input;
  }
  sendCallback();
}

void SonarConfiguration::setGamma(double input) {
  if (input <= 255 && input > 0) {
    _sfm.gammaCorrection = input;
  }
  sendCallback();
}

void SonarConfiguration::setPingRate(PingRateType newRate) {
  _sfm.pingRate = newRate;
  sendCallback();
}

void SonarConfiguration::setFreqMode(OculusFreqMode input) {
  _sfm.masterMode = input;
  sendCallback();
}

void SonarConfiguration::serializeTo(boost::asio::streambuf &stream) const {
  stream.sputn(reinterpret_cast<const char*>(&_sfm),
               sizeof(OculusSimpleFireMessage));
}

}  // namespace liboculus
