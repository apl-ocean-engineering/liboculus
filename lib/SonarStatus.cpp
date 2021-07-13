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

#include "liboculus/SonarStatus.h"

#include "g3log/g3log.hpp"


namespace liboculus {

using std::string;

using boost::asio::ip::address_v4;

SonarStatus::SonarStatus()
    : _statusMutex(),
      _valid(false) {}

OculusStatusMsg SonarStatus::operator()(void) const {
  std::lock_guard<std::mutex> lock(_statusMutex);

  return _osm;
}

boost::asio::ip::address SonarStatus::ipAddr() const {
  std::lock_guard<std::mutex> lock(_statusMutex);
  return address_v4(ntohl(_osm.ipAddr));
}

void SonarStatus::update(const OculusStatusMsg &msg,  sys_time_point msgTime) {
  {
    std::lock_guard<std::mutex> lock(_statusMutex);
    memcpy((void *)&_osm, (void *)&msg, sizeof(OculusStatusMsg));

    _valid = true;  // TODO:  Should actually validate contents
    _msgTime = msgTime;
  }

  // Dump must be outside of the lock_guard or you get a race condition
  //dump();
}

void SonarStatus::dump() const {
  std::lock_guard<std::mutex> lock(_statusMutex);

  LOG(DEBUG) << "Device id " << _osm.deviceId << " ; type: " <<  (uint16_t)_osm.deviceType << " ; part num: " << (uint16_t)_osm.partNumber;

  //LOG(DEBUG) << "        Received at: " << _msgTime;
  LOG(DEBUG) << "             Status: " << std::hex << _osm.status;
  LOG(DEBUG) << "      Sonar ip addr: " << boost::asio::ip::address_v4(ntohl(_osm.ipAddr));
  LOG(DEBUG) << " Sonar connected to: " << boost::asio::ip::address_v4(ntohl(_osm.connectedIpAddr));

  LOG(DEBUG) << "Versions:";
  LOG(DEBUG) << "   firmwareVersion0: " << std::hex << _osm.versionInfo.firmwareVersion0;
  LOG(DEBUG) << "      firmwareDate0: " << std::hex << _osm.versionInfo.firmwareDate0;

  LOG(DEBUG) << "   firmwareVersion1: " << std::hex << _osm.versionInfo.firmwareVersion1;
  LOG(DEBUG) << "      firmwareDate1: " << std::hex << _osm.versionInfo.firmwareDate1;

  LOG(DEBUG) << "   firmwareVersion2: " << std::hex << _osm.versionInfo.firmwareVersion2;
  LOG(DEBUG) << "      firmwareDate2: " << std::hex << _osm.versionInfo.firmwareDate2;
}

}  // namespace liboculus
