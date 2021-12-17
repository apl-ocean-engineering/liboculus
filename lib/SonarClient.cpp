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

namespace asio = boost::asio;


SonarClient::SonarClient(const std::string &ipAddr)
    : _ioSrv(),
      _socket(_ioSrv.service()),
      _statusRx(_ioSrv.service())
{
  _statusRx.setCallback(std::bind(&SonarClient::receiveStatus, this,
                                  std::placeholders::_1));

  if (!ipAddr.empty() && ipAddr != "auto") {
    LOG(INFO) << "Connecting to sonar with IP address " << ipAddr;
    auto addr(boost::asio::ip::address_v4::from_string(ipAddr));

    LOG_IF(FATAL,addr.is_unspecified()) << "Couldn't parse IP address" << ipAddr;
    connect(addr);
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

  if(connected()) return;

  // Attempt to connect to the auto-detected unit
  if(status.valid()) {
    const auto addr(status.ipAddr());
    LOG(INFO) << "Using sonar detected at " << addr;
    connect(addr);
  }
}


void SonarClient::connect(const asio::ip::address &addr) {
  if (connected()) return;

  uint16_t port = 52100;
  asio::ip::address ipAddress = addr;

  boost::asio::ip::tcp::endpoint sonarEndpoint(ipAddress, port);

  LOG(DEBUG) << "Connecting to sonar at " << sonarEndpoint;

  // config.setCallback(std::bind(&DataRx::sendConfiguration, this,
  //                              std::placeholders::_1));

  // Kick off the first async_io when the socket connects
  _socket.async_connect(sonarEndpoint,
                        boost::bind(&SonarClient::onConnect, this, _1));
}


void SonarClient::onConnect(const boost::system::error_code& ec) {
  if (!ec) {
    scheduleHeaderRead();

    _onConnectCallback();
  } else {
    LOG(WARNING) << "Error on connect: " << ec.message();
  }
}

//== Data writers

void SonarClient::sendConfiguration(const SonarConfiguration &msg) {
  std::vector<std::uint8_t> vector_buffer = msg.serialize();
  asio::const_buffer vector_buffer_view = asio::buffer(vector_buffer);

  auto result = _socket.send(vector_buffer_view);
  LOG(DEBUG) << "Sent " << result << " bytes to sonar";

  _dataTxCallback(vector_buffer);
}

//=== Readers
void SonarClient::scheduleHeaderRead() {
  MessageHeader header;

  _socket.async_receive(boost::asio::buffer((void *)header.ptr(),
                                            sizeof(OculusMessageHeader)),
                        boost::bind(&SonarClient::readHeader, this, header, _1, _2));
}


void SonarClient::readHeader(MessageHeader hdr, const boost::system::error_code& ec,
                        std::size_t bytes_transferred) {
  if (ec) {
    LOG(WARNING) << "Error on receive of header: " << ec.message();
    return;
  }
  LOG(DEBUG) << "Got " << bytes_transferred << " bytes of header from sonar";

 // _dataRxCallback(header.buffer());

  if (bytes_transferred != sizeof(OculusMessageHeader)) {
    LOG(WARNING) << "Received short header of " << bytes_transferred
                 << " expected " << sizeof(OculusMessageHeader);
    return;
  }
  LOG(DEBUG) << "Validating...";

  if (!hdr.valid()) {
    LOG(WARNING) << "Incoming header invalid";
    return;
  }

  LOG(DEBUG) << "Got message ID " << static_cast<int>(hdr.msgId());
  // Possible options for msgId() are:
  // * messageSimpleFire
  // * messagePingResult
  // * messageSimplePingResult
  // * messageUserConfig
  // * messageLogs
  // * messageDummy

  // TODO(lindzey): This seems to guarantee a buffer overrun if we just continue here.
  if (hdr.msgId() == messageSimplePingResult) {
    if (!hdr.expandForPayload()) {
      LOG(WARNING) << "Unable to expand for payload";
    }

    // Read the remainder of the packet
    // QUESTION(lindzey): Why is this scheduled as an async_read, while the
    //    others directly read?
    auto b = boost::asio::buffer(hdr.payloadPtr(), hdr.payloadSize());
    boost::asio::async_read(_socket, b,
                            boost::bind(&SonarClient::readSimplePingResult,
                                        this, hdr, _1, _2));

    // readSimplePingResult will call scheduleHeaderRead(), so don't
    // call it here.
    // QUESTION(LEL): However, scheduleHeaderRead is only called on a
    //     *successful* read of a SimplePingResult (and in general, only
    //     after successful parsing of the previous message ... will this
    //     cause problems? Is it maybe why I have to restart the driver
    //     twice to recover after getting it into a weird state?

  } else {
    // Always download the rest of the message.
    auto payload_bytes = hdr.payloadSize();
    uint32_t bytes_received;
    boost::asio::streambuf junk_buffer(hdr.payloadSize());
    if (payload_bytes > 0) {
      LOG(DEBUG) << "Fetching " << payload_bytes << " bytes of payload";
      // Q(lindzey): Is it OK for this NOT to be an async_read? It replaces
      //   multiple calls, some of which were read and the rest async_read.
      bytes_received = boost::asio::read(_socket, junk_buffer);
      if (bytes_received != payload_bytes) {
        LOG(WARNING) << "Requested " << payload_bytes << " payload bytes, "
                     << "but only received " << bytes_received;
      }
    }

 // _dataRxCallback(junk_buffer.data());

    // Message-specific handling
    if (hdr.msgId() == messageLogs) {
      // Actually want to log these!
      LOG(DEBUG) << "Read " << bytes_received << " of logging info";
      if (bytes_received > 0) {
        std::string s((std::istreambuf_iterator<char>(&junk_buffer)),
                      std::istreambuf_iterator<char>());
        LOG(DEBUG) << s;
      } else {
        LOG(WARNING) << "Error on receive of payload for log message: "
                     << ec.message();
      }
    } else if (hdr.msgId() == messageDummy) {
      LOG(DEBUG) << "Ignoring dummy message";
    } else {
      // Unhandled values of the OculusMessageType enum:
      // messagesSimpleFire, messagePingResult, messageUserConfig
      LOG(INFO) << "Unknown message ID " << static_cast<int>(hdr.msgId());
    }

    // Finally, set up for the next round.
    scheduleHeaderRead();
  }
}

void SonarClient::readSimplePingResult(MessageHeader hdr,
                                  const boost::system::error_code& ec,
                                  std::size_t bytes_transferred) {
  if (ec) {
    LOG(WARNING) << "Error on receive of simplePingResult: " << ec.message();
    return;
  }
  LOG(DEBUG) << "Got " << bytes_transferred
             << " bytes of SimplePingResult from sonar";

  if (bytes_transferred != hdr.payloadSize()) {
    LOG(WARNING) << "Received short header of " << bytes_transferred
                 << " expected " << hdr.payloadSize();
    return;
  }

  SimplePingResult ping(hdr);

  if (ping.valid()) {
    LOG(DEBUG) << "Data valid!";
    _simplePingCallback(ping);

    // And return to the home state
    scheduleHeaderRead();
  } else {
    LOG(WARNING) << "Incoming packet invalid";
  }
}

}  // namespace liboculus
