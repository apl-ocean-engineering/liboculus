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

#include "liboculus/DataRx.h"

namespace liboculus {

namespace asio = boost::asio;


DataRx::DataRx(const std::shared_ptr<IoServiceThread> &iosrv)
    : _ioSrv(iosrv),
      _socket(iosrv->service())
{
}

DataRx::~DataRx() {
}

void DataRx::connect(const asio::ip::address &addr) {
  if (connected()) return;

  uint16_t port = 52100;

  boost::asio::ip::tcp::endpoint sonarEndpoint(addr, port);

  LOG(DEBUG) << "Connecting to sonar at " << sonarEndpoint;

  _socket.async_connect(sonarEndpoint,
                        boost::bind(&DataRx::onConnect, this, _1));
}


void DataRx::onConnect(const boost::system::error_code& ec) {
  if (!ec) {
    scheduleHeaderRead();
    _onConnectCallback();
  } else {
    LOG(WARNING) << "Error on connect: " << ec.message();
  }
}

//== Data writers

void DataRx::sendSimpleFireMessage(const SonarConfiguration &msg) {
  std::vector<std::uint8_t> vector_buffer = msg.serialize();
  asio::const_buffer vector_buffer_view = asio::buffer(vector_buffer);

  auto result = _socket.send(vector_buffer_view);
  LOG(DEBUG) << "Sent " << result << " bytes to sonar";

  //_dataTxCallback(vector_buffer);
}

//=== Readers
void DataRx::scheduleHeaderRead() {
  MessageHeader header;

  _socket.async_receive(boost::asio::buffer((void *)header.ptr(),
                                            sizeof(OculusMessageHeader)),
                        boost::bind(&DataRx::readHeader, this, header, _1, _2));
}


void DataRx::readHeader(MessageHeader hdr, const boost::system::error_code& ec,
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
                            boost::bind(&DataRx::readSimplePingResult,
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

void DataRx::readSimplePingResult(MessageHeader hdr,
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
