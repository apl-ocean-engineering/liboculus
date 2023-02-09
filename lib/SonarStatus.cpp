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

#include "liboculus/SonarStatus.h"

#include "g3log/g3log.hpp"

namespace liboculus {

using std::string;

using boost::asio::ip::address_v4;

SonarStatus::SonarStatus(const ByteVector &buffer) : _buffer(buffer) {}

boost::asio::ip::address SonarStatus::ipAddr() const {
  return address_v4(ntohl(msg()->ipAddr));
}

void SonarStatus::dump() const {
  LOG(DEBUG) << "Device id " << msg()->deviceId
             << " ; type: " << (uint16_t)msg()->deviceType
             << " ; part num: " << (uint16_t)msg()->partNumber;

  // LOG(DEBUG) << "        Received at: " << _msgTime;
  LOG(DEBUG) << "             Status: " << std::hex << msg()->status;
  LOG(DEBUG) << "      Sonar ip addr: "
             << boost::asio::ip::address_v4(ntohl(msg()->ipAddr));
  LOG(DEBUG) << " Sonar connected to: "
             << boost::asio::ip::address_v4(ntohl(msg()->connectedIpAddr));

  LOG(DEBUG) << "Versions:";
  LOG(DEBUG) << "   firmwareVersion0: " << std::hex
             << msg()->versionInfo.firmwareVersion0;
  LOG(DEBUG) << "      firmwareDate0: " << std::hex
             << msg()->versionInfo.firmwareDate0;

  LOG(DEBUG) << "   firmwareVersion1: " << std::hex
             << msg()->versionInfo.firmwareVersion1;
  LOG(DEBUG) << "      firmwareDate1: " << std::hex
             << msg()->versionInfo.firmwareDate1;

  LOG(DEBUG) << "   firmwareVersion2: " << std::hex
             << msg()->versionInfo.firmwareVersion2;
  LOG(DEBUG) << "      firmwareDate2: " << std::hex
             << msg()->versionInfo.firmwareDate2;
}

}  // namespace liboculus
