// Copyright (c) 2017-2026 University of Washington
// Author: Aaron Marburg <amarburg@uw.edu>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of University of Washington nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <boost/asio.hpp>
#include <fmt/format.h>
#include <string>
#include <vector>

#include "liboculus/SonarStatus.h"

namespace liboculus {

using std::string;

using boost::asio::ip::address_v4;

SonarStatus::SonarStatus(const ByteVector &buffer) : _buffer(buffer) {}

boost::asio::ip::address SonarStatus::ipAddr() const {
  return address_v4(ntohl(msg()->ipAddr));
}

std::vector<std::string>
SonarStatus::dump(std::vector<std::string> &vec) const {
  vec.push_back(fmt::format("Device id: {}; type: {}; part num: {}",
                            msg()->deviceId,
                            static_cast<uint16_t>(msg()->deviceType),
                            static_cast<uint16_t>(msg()->partNumber)));

  // vec.push_back( fmt::format("        Received at: {}", _msgTime));
  vec.push_back(fmt::format("             Status: {:#04x}", msg()->status));
  vec.push_back(fmt::format(
      "      Sonar ip addr: {}",
      boost::asio::ip::address_v4(ntohl(msg()->ipAddr)).to_string()));
  vec.push_back(fmt::format(
      " Sonar connected to: {}",
      boost::asio::ip::address_v4(ntohl(msg()->connectedIpAddr)).to_string()));

  vec.push_back(fmt::format("   firmwareVersion0: {:#04x}",
                            msg()->versionInfo.firmwareVersion0));
  vec.push_back(fmt::format("      firmwareDate0: {:#04x}",
                            msg()->versionInfo.firmwareDate0));

  vec.push_back(fmt::format("   firmwareVersion1: {:#04x}",
                            msg()->versionInfo.firmwareVersion1));
  vec.push_back(fmt::format("      firmwareDate1: {:#04x}",
                            msg()->versionInfo.firmwareDate1));

  vec.push_back(fmt::format("   firmwareVersion2: {:#04x}",
                            msg()->versionInfo.firmwareVersion2));
  vec.push_back(fmt::format("      firmwareDate2: {:#04x}",
                            msg()->versionInfo.firmwareDate2));

  return vec;
}

} // namespace liboculus
