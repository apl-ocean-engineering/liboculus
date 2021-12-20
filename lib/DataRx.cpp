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

#include <boost/bind.hpp>

#include "liboculus/DataRx.h"

namespace liboculus {

namespace asio = boost::asio;


DataRx::DataRx(const std::shared_ptr<boost::asio::io_context> &iosrv)
    : _socket(*iosrv),
      _simplePingCallback([](const SimplePingResult &){}),
      _onConnectCallback([](void){}) {
}

DataRx::~DataRx() {
}

void DataRx::connect(const asio::ip::address &addr) {
  if (isConnected()) return;

  uint16_t port = 52100;

  boost::asio::ip::tcp::endpoint sonarEndpoint(addr, port);
  LOG(INFO) << "Connecting to sonar at " << sonarEndpoint;

  _socket.async_connect(sonarEndpoint,
                        boost::bind(&DataRx::onConnect, this, _1));
}


void DataRx::onConnect(const boost::system::error_code& ec) {
  if (ec) {
    LOG(WARNING) << "Error on connect: " << ec.message();
  }

  LOG(INFO) << "Connected to sonar!";
  scheduleHeaderRead();
  _onConnectCallback();
}

//== Data writers

void DataRx::sendSimpleFireMessage(const SonarConfiguration &msg) {
  if (!isConnected()) {
    LOG(WARNING) << "Can't send to sonar, not connected";
    return;
  }

  std::vector<std::uint8_t> data = msg.serialize();

  auto result = _socket.send(asio::buffer(data));
  LOG(INFO) << "Sent " << result << " bytes to sonar";

  haveWritten(data);
}
  

//=== Readers
void DataRx::readUpTo(size_t bytes,
                    std::function<void(const boost::system::error_code&,std::size_t)> callback) {
  const size_t current_sz = _buffer.size();
  _buffer.resize(bytes);

  LOG(INFO) << " Socket " << (_socket.is_open() ? "open" : "closed") << " with " << _socket.available();
  LOG(INFO) << "Resizing buffer to " << _buffer.size() << " bytes";
  asio::mutable_buffer buffer_view = asio::buffer(_buffer)+current_sz;
  LOG(INFO) <<" Waiting for " << buffer_size(buffer_view) << " bytes";
  //async_read(_socket, buffer_view, boost::asio::transfer_all(), callback);
  asio::async_read(_socket, buffer_view, boost::asio::transfer_exactly(bytes-current_sz), callback);
}

void DataRx::scheduleHeaderRead() {
  LOG(INFO) << "== Back to start of state machine ==";

  _buffer.clear();

  readUpTo(sizeof(uint8_t),
          boost::bind(&DataRx::rxFirstByteOculusId, this, _1, _2));
}


//==== States in the state machine... ====

void DataRx::rxFirstByteOculusId(const boost::system::error_code& ec,
                        std::size_t bytes_transferred) {  
  if (ec) {
    LOG(WARNING) << "Error on receive of header: " << ec.message();
    scheduleHeaderRead();
  }

  if (bytes_transferred != sizeof(uint8_t)) {
    scheduleHeaderRead();
  }

  LOG(WARNING) << "Read " << bytes_transferred << " bytes : " << std::hex << static_cast<int>(_buffer[0]);

  if (_buffer.data()[0] == 0x53) {
    readUpTo(sizeof(uint16_t),
              boost::bind(&DataRx::rxSecondByteOculusId, this, _1, _2));
    return;
  }

  scheduleHeaderRead();
}

void DataRx::rxSecondByteOculusId(const boost::system::error_code& ec,
                        std::size_t bytes_transferred) {  
  if (ec) {
    LOG(WARNING) << "Error on receive of header: " << ec.message();
    scheduleHeaderRead();
  }

  if (bytes_transferred != sizeof(uint8_t)) {
    scheduleHeaderRead();
  }

  LOG(WARNING) << "Read " << bytes_transferred << " bytes : " << std::hex << static_cast<int>(_buffer[1]);

  if (_buffer.data()[1] == 0x4f) {
    LOG(WARNING) << "Got OculusId at start of packet";

    readUpTo(sizeof(OculusMessageHeader),
              boost::bind(&DataRx::rxHeader, this, _1, _2));
    return;
  }

  scheduleHeaderRead();
}

void DataRx::rxHeader(const boost::system::error_code& ec,
                        std::size_t bytes_transferred) {  
  if (ec) {
    LOG(WARNING) << "Error on receive of header: " << ec.message();
    return;
  }
  LOG(INFO) << "Got " << bytes_transferred << " bytes of header from sonar";

 // _dataRxCallback(header.buffer());

if (bytes_transferred != (sizeof(OculusMessageHeader)-sizeof(uint16_t))) {
     LOG(WARNING) << "Received short header of " << bytes_transferred
                  << " expected " << sizeof(OculusMessageHeader);
    scheduleHeaderRead();
  }

  MessageHeader hdr(_buffer);

  LOG(WARNING) << "Validating OculusMessageHeader...";

  if (!hdr.valid()) {
    LOG(WARNING) << "Incoming header invalid";
    scheduleHeaderRead();
    return;
  }

  LOG(INFO) << "Got message ID " <<  static_cast<int>(hdr.msgId()) << " (" << MessageTypeToString(hdr.msgId()) << ")";
  // Possible options for msgId() are:
  // * messageSimpleFire
  // * messagePingResult
  // * messageSimplePingResult
  // * messageUserConfig
  // * messageLogs
  // * messageDummy

  hdr.dump();

  const auto packetSize = hdr.packetSize();
  const auto id = hdr.msgId();
  if ((id == messageSimpleFire) || 
      (id == messagePingResult) ||
      (id == messageUserConfig) ||
      (id == messageDummy)) {
    // I think these messages are exclusively user -> sonar
    // so we should never receive them
    readUpTo(packetSize,
              boost::bind(&DataRx::rxIgnoredData, this, _1, _2));
  } else if (id == messageSimplePingResult) {
    readUpTo(packetSize,
              boost::bind(&DataRx::rxSimplePingResult, this, _1, _2));
  } else if (id == messageLogs) {
    readUpTo(packetSize,
              boost::bind(&DataRx::rxMessageLogs, this, _1, _2));
  } else {
    LOG(WARNING) << "Not sure how to handle message ID " << static_cast<int>(hdr.msgId());
    scheduleHeaderRead();
  }

  // If all else fails, just start again


//   // TODO(lindzey): This seems to guarantee a buffer overrun if we just continue here.
//   if (hdr.msgId() == messageSimplePingResult) {
//     if (!hdr.expandForPayload()) {
//       LOG(WARNING) << "Unable to expand for payload";
//     }

//     // Read the remainder of the packet
//     // QUESTION(lindzey): Why is this scheduled as an async_read, while the
//     //    others directly read?
//     auto b = boost::asio::buffer(hdr.payloadPtr(), hdr.payloadSize());
//     boost::asio::async_read(_socket, b,
//                             boost::bind(&DataRx::readSimplePingResult,
//                                         this, hdr, _1, _2));

//     // readSimplePingResult will call scheduleHeaderRead(), so don't
//     // call it here.
//     // QUESTION(LEL): However, scheduleHeaderRead is only called on a
//     //     *successful* read of a SimplePingResult (and in general, only
//     //     after successful parsing of the previous message ... will this
//     //     cause problems? Is it maybe why I have to restart the driver
//     //     twice to recover after getting it into a weird state?

//   } else {
//     // Always download the rest of the message.
//     auto payload_bytes = hdr.payloadSize();
//     uint32_t bytes_received;
//     boost::asio::streambuf junk_buffer(hdr.payloadSize());
//     if (payload_bytes > 0) {
//       LOG(DEBUG) << "Fetching " << payload_bytes << " bytes of payload";
//       // Q(lindzey): Is it OK for this NOT to be an async_read? It replaces
//       //   multiple calls, some of which were read and the rest async_read.
//       bytes_received = boost::asio::read(_socket, junk_buffer);
//       if (bytes_received != payload_bytes) {
//         LOG(WARNING) << "Requested " << payload_bytes << " payload bytes, "
//                      << "but only received " << bytes_received;
//       }
//     }

//  // _dataRxCallback(junk_buffer.data());

//     // Message-specific handling
//     if (hdr.msgId() == messageLogs) {
//       // Actually want to log these!
//       LOG(INFO) << "Read " << bytes_received << " of logging info";
//       if (bytes_received > 0) {
//         std::string s((std::istreambuf_iterator<char>(&junk_buffer)),
//                       std::istreambuf_iterator<char>());
//         LOG(INFO) << s;
//       } else {
//         LOG(WARNING) << "Error on receive of payload for log message: "
//                      << ec.message();
//       }
//     } else if (hdr.msgId() == messageDummy) {
//       LOG(INFO) << "Ignoring dummy message";
//     } else {
//       // Unhandled values of the OculusMessageType enum:
//       // messagesSimpleFire, messagePingResult, messageUserConfig
//       LOG(INFO) << "Unknown message ID " << static_cast<int>(hdr.msgId());
//     }

//     // Finally, set up for the next round.
//     scheduleHeaderRead();
//   }
}

void DataRx::rxSimplePingResult(const boost::system::error_code& ec,
                                  std::size_t bytes_transferred) {
  LOG(WARNING) << "Got simple ping result!!";

  if (ec) {
    LOG(WARNING) << "Error on receive of simplePingResult: " << ec.message();
    scheduleHeaderRead();
  }

  LOG(INFO) << "Got " << bytes_transferred
            << " bytes of SimplePingResult from sonar";

  // if (bytes_transferred != hdr.payloadSize()) {
  //   LOG(WARNING) << "Received short header of " << bytes_transferred
  //                << " expected " << hdr.payloadSize();
  //   return;
  // }

  SimplePingResult ping(_buffer);

  ping.dump();


  if (ping.valid()) {
    LOG(INFO) << "Header valid!";

    if (bytes_transferred < ping.payloadSize()) {
      LOG(WARNING) << "Did not read full data packet, resetting...";
      // readUpTo(ping.payloadSize(),
      //   boost::bind(&DataRx::rxSimplePingResult, this, _1, _2));
      // return;
    }
  //   _simplePingCallback(ping);

  // And return to the home state
  } else {
    LOG(WARNING) << "Incoming packet invalid";
  }

  scheduleHeaderRead();
}

void DataRx::rxMessageLogs(const boost::system::error_code& ec,
                                  std::size_t bytes_transferred) {
  if (ec) {
    LOG(WARNING) << "Error on receive of rxMessageLogs: " << ec.message();
    scheduleHeaderRead();
  }

  LOG(INFO) << "Received " << bytes_transferred << " of LogMessage data";

  LOG(INFO) << "Buffer is " << _buffer.size() << " bytes in length";

  LOG(INFO) << std::string(_buffer.begin()+sizeof(OculusMessageHeader), _buffer.end());
  scheduleHeaderRead();
}

void DataRx::rxIgnoredData(const boost::system::error_code& ec,
                                  std::size_t bytes_transferred) {
  if (ec) {
    LOG(WARNING) << "Error on receive of rxIgnoredData: " << ec.message();
    scheduleHeaderRead();
  }

  LOG(INFO) << "Ignoring " << bytes_transferred << " of data";
  scheduleHeaderRead();
}

}  // namespace liboculus
