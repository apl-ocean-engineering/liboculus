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

#include "liboculus/SonarClient.h"

namespace liboculus {


SonarClient::SonarClient(SonarConfiguration &config, const std::string &ipAddr)
    : _ioSrv(),
      _statusRx(_ioSrv.service()),
      _dataRx(_ioSrv.service()),
      _config(config) {

  _statusRx.setCallback(std::bind(&SonarClient::receiveStatus, this,
                                  std::placeholders::_1));

  if (!ipAddr.empty() && ipAddr != "auto") {
    LOG(INFO) << "Connecting to sonar with IP address " << ipAddr;
    auto addr(boost::asio::ip::address_v4::from_string(ipAddr));

    LOG_IF(FATAL,addr.is_unspecified()) << "Couldn't parse IP address" << ipAddr;
    _dataRx.connect(addr, _config);
  }
}

SonarClient::~SonarClient() {
  stop();
  join();
}

void SonarClient::start() {
  _ioSrv.fork();
}

void SonarClient::join() {
  _ioSrv.join();
}

void SonarClient::stop() {
  _ioSrv.stop();
}

// TODO(lindzey): Should any of these get surfaced to ROS?
void SonarClient::receiveStatus(const SonarStatus &status) {
  // Always check the sonar status
  {
    uint32_t status_flags = status.status();

    // Lifted from the example SDK
    OculusMasterStatusType mst = (OculusMasterStatusType)(status_flags & 0x07);
    bool checkPause = false;

    if (mst == oculusMasterStatusSsblBoot) {
      LOG(WARNING) << "Error: SSBL Bootloader";
      checkPause = true;
    } else if (mst == oculusMasterStatusSsblRun) {
      LOG(WARNING) << "Error: SSBL Run";
      checkPause = true;
    }
    /*
    else if (mst == oculusMasterStatusMainBoot) {
      LOG(WARNING) << "Error: Main Bootloader";
    } else if (mst == oculusMasterStatusMainRun) {
      LOG(WARNING) << "Error: Main Run";
    }
    */

    // Check the pause reason
    if (checkPause) {
      OculusPauseReasonType prt = (OculusPauseReasonType)((status_flags & 0x38) >> 3);

      if (prt == oculusPauseMagSwitch) {
        LOG(WARNING) << "Halt: Mag Switch Detected";
      } else if (prt == oculusPauseBootFromMain) {
        LOG(WARNING) << "Halt: Boot From Main";
      } else if (prt == oculusPauseFlashError) {
        LOG(WARNING) << "Halt: Flash Error. Update firmware";
      } else if (prt == oculusPauseJtagLoad) {
        LOG(WARNING) << "Halt: JTAG Load";
      }
    }

    // High temp
    const bool overTempShutdown = (status_flags & (1 << 15));
    const bool highTemp = (status_flags & (1 << 14));

    if (overTempShutdown) {
      LOG(WARNING) << "Warning: High Temp - Pings Stopped";
    } else if (highTemp) {
      LOG(WARNING) << "Warning: High Temperature";
    }

    const bool transmitError = (status_flags & (1 << 16));
    if (transmitError) {
      LOG(WARNING) << "Critical: Transmit Circuit Failure";
    }
  }

  if(_dataRx.connected()) return;

  status.dump();

  // Attempt auto detection
  if(status.valid()) {
    auto addr(status.ipAddr());
    LOG(INFO) << "Using sonar detected at " << addr;
    _dataRx.connect(addr, _config);
  }
}

}  // namespace liboculus
