/*
 * Copyright (c) 2017-2025 University of Washington
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

#include "liboculus/StatusRx.h"
#include "liboculus/Logger.h"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#endif
#include <string.h>

#include <iomanip>
#include <sstream>

#include "liboculus/Constants.h"

namespace liboculus {

using boost::asio::ip::address_v4;
using std::string;

// ----------------------------------------------------------------------------
// StatusRx - a listening socket for oculus status messages

StatusRx::StatusRx(const IoServiceThread::IoContextPtr &iosrv)
    : _num_valid_rx(0), _num_invalid_rx(0), _socket(*iosrv), _deadline(*iosrv),
      _sonarStatusCallback([](const SonarStatus &, bool) {}) {
  doConnect();
}

void StatusRx::doConnect() {
  boost::asio::ip::udp::endpoint local(boost::asio::ip::address_v4::any(),
                                       StatusBroadcastPort);

  boost::system::error_code error;
  _socket.open(boost::asio::ip::udp::v4(), error);
  if (error) {
    oclog::error("StatusRx open failed: {} ({})", error.message(),
                 error.value());
    return;
  }

  boost::asio::socket_base::broadcast option(true);
  _socket.set_option(option, error);
  if (error) {
    oclog::error("StatusRx set broadcast failed: {} ({})", error.message(),
                 error.value());
    return;
  }

  boost::asio::socket_base::reuse_address reuse(true);
  _socket.set_option(reuse, error);
  if (error) {
    oclog::error("StatusRx set reuse_address failed: {} ({})", error.message(),
                 error.value());
    return;
  }

  _socket.bind(local, error);
  if (error) {
    oclog::error("StatusRx bind failed: {} ({})", error.message(),
                 error.value());
    return;
  }

  scheduleRead();
}

void StatusRx::scheduleRead() {
  // Start an asynchronous receive
  _buffer.resize(sizeof(OculusStatusMsg));
  oclog::trace("Waiting for status packet...");
  _socket.async_receive(boost::asio::buffer(_buffer),
                        std::bind(&StatusRx::handleRead, this,
                                  std::placeholders::_1,
                                  std::placeholders::_2));
}

void StatusRx::handleRead(const boost::system::error_code &ec,
                          std::size_t bytes_transferred) {
  if (ec) {
    oclog::warn("Error on receive: {}", ec.message());
    scheduleRead();
    // Don't try to parse a partial buffer after a receive error.
    return;
  }

  oclog::info("StatusRx received {} bytes", bytes_transferred);

  if (bytes_transferred != sizeof(OculusStatusMsg)) {
    oclog::warn("Got {} bytes, expected OculusStatusMsg of size {}",
                bytes_transferred, sizeof(OculusStatusMsg));
    _num_invalid_rx++;
    scheduleRead();
    return;
  }

  SonarStatus status(_buffer);

  // Uncomment to dump every status packet
  // std::vector<std::string> dump_vec;
  // status.dump(dump_vec);

  // for (auto const &l : dump_vec) {
  //   oclog::info("Status: {}", l);
  // }

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
      oclog::warn("Error: SSBL Bootloader");
      checkPause = true;
    } else if (mst == oculusMasterStatusSsblRun) {
      oclog::warn("Error: SSBL Run");
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
      OculusPauseReasonType prt =
          (OculusPauseReasonType)((status_flags & 0x38) >> 3);

      if (prt == oculusPauseMagSwitch) {
        oclog::error("Halt: Mag Switch Detected");
      } else if (prt == oculusPauseBootFromMain) {
        oclog::error("Halt: Boot From Main");
      } else if (prt == oculusPauseFlashError) {
        oclog::error("Halt: Flash Error. Update firmware");
      } else if (prt == oculusPauseJtagLoad) {
        oclog::error("Halt: JTAG Load");
      } else if (prt == oculusPauseFirmwareError) {
        oclog::error("Halt: Firmware error");
      } else if (prt == oculusPauseCompatibilityError) {
        oclog::error("Halt: Compatibility error");
      } else if (prt == oculusPauseBrownout) {
        oclog::error("Halt: Brownout");
      } else {
        oclog::error("Halt: unknown error ({:#4x})", static_cast<int>(prt));
      }

      return false;
    }

    // High temp
    const bool overTempShutdown = (status_flags & (1 << 15));
    const bool highTemp = (status_flags & (1 << 14));

    if (overTempShutdown) {
      oclog::error("Warning: High Temp - Pings Stopped");
      return false;
    } else if (highTemp) {
      oclog::error("Warning: High Temperature");
    }

    const bool transmitError = (status_flags & (1 << 16));
    if (transmitError) {
      oclog::error("Critical: Transmit Circuit Failure");
      return false;
    }
  }

  return true;
}

} // namespace liboculus
