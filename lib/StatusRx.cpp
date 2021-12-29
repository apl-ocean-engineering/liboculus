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

#include <string.h>
#include <sstream>

#include <arpa/inet.h>

#include <boost/bind.hpp>

#include "liboculus/StatusRx.h"
#include "liboculus/Constants.h"
#include "g3log/g3log.hpp"

namespace liboculus {

using std::string;
using boost::asio::ip::address_v4;

// ----------------------------------------------------------------------------
// StatusRx - a listening socket for oculus status messages

StatusRx::StatusRx(const IoServiceThread::IoContextPtr &iosrv)
    : _num_valid_rx(0),
      _num_invalid_rx(0),
      _socket(*iosrv),
      _deadline(*iosrv),
      _sonarStatusCallback([](const SonarStatus &, bool){}) {
  doConnect();
}

void StatusRx::doConnect() {
  boost::asio::ip::udp::endpoint local(boost::asio::ip::address_v4::any(),
                                        StatusBroadcastPort);

  boost::system::error_code error;
  _socket.open(boost::asio::ip::udp::v4(), error);

  boost::asio::socket_base::broadcast option(true);
  _socket.set_option(option);

  if (!error) {
    _socket.bind(local);
    scheduleRead();
  } else {
    LOG(WARNING) << "Unable to start reader";
  }
}

void StatusRx::scheduleRead() {
  // Start an asynchronous receive
  _buffer.resize(sizeof(OculusStatusMsg));
  LOG(DEBUG) << "Waiting for status packet...";
  _socket.async_receive(boost::asio::buffer(_buffer),
                        boost::bind(&StatusRx::handleRead, this, _1, _2));
}

void StatusRx::handleRead(const boost::system::error_code& ec,
                          std::size_t bytes_transferred) {
  if (ec) {
    LOG(WARNING) << "Error on receive: " << ec.message();
    scheduleRead();
  }

  LOG(DEBUG) << "Read " << bytes_transferred << " bytes";

  if (bytes_transferred != sizeof(OculusStatusMsg)) {
      LOG(WARNING) << "Got " << bytes_transferred
                    << " bytes, expected OculusStatusMsg of size "
                    << sizeof(OculusStatusMsg);
      _num_invalid_rx++;
      return;
    }

  SonarStatus status(_buffer);
  status.dump();
  auto is_good = parseStatus(status);

  if (_sonarStatusCallback) {
    _sonarStatusCallback(status, is_good);
  }

  _num_valid_rx++;
  scheduleRead();
}

// TODO(lindzey): Should any of these get surfaced to ROS?
bool StatusRx::parseStatus(const SonarStatus &status) {
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

      return false;
    }

    // High temp
    const bool overTempShutdown = (status_flags & (1 << 15));
    const bool highTemp = (status_flags & (1 << 14));

    if (overTempShutdown) {
      LOG(WARNING) << "Warning: High Temp - Pings Stopped";
      return false;
    } else if (highTemp) {
      LOG(WARNING) << "Warning: High Temperature";
    }

    const bool transmitError = (status_flags & (1 << 16));
    if (transmitError) {
      LOG(WARNING) << "Critical: Transmit Circuit Failure";
      return false;
    }
  }

  return true;
}

}  // namespace liboculus
