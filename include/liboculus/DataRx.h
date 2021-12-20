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

#pragma once

#include <string>
#include <thread>
#include <vector>
#include <memory>

#include "liboculus/OculusStructs.h"
#include "liboculus/SonarConfiguration.h"

namespace liboculus {

class DataRx {
 public:
  explicit DataRx(const std::shared_ptr<boost::asio::io_context> &iosrv);

  ~DataRx();

  void connect(const boost::asio::ip::address &addr);
  void connect(const std::string &strAddr) {
       auto addr(boost::asio::ip::address_v4::from_string(strAddr));
       //LOG_IF(FATAL,addr.is_unspecified()) << "Couldn't parse IP address" << ipAddr;  
       connect(addr);
  }

  bool isConnected() const { return _socket.is_open(); }

  typedef std::function< void(const SimplePingResult &) > SimplePingCallback;
  void setSimplePingCallback(SimplePingCallback callback) {
    _simplePingCallback = callback;
  }

  typedef std::function< void() > OnConnectCallback;
  void setOnConnectCallback(OnConnectCallback callback) {
    _onConnectCallback = callback;
  }

  void sendSimpleFireMessage(const SonarConfiguration &config);

  // Implement data read / data written hooks as virtual functions rather
  // rather than callbacks.
  virtual void haveWritten(const std::vector<uint8_t> &bytes) {;}
  virtual void haveRead(const std::vector<uint8_t> &bytes) {;}

 private:
  void onConnect(const boost::system::error_code& error);
 
  // Initiates another network read.
  // Note this function reads until the **total number of bytes
  // in _buffer == bytes**   The actual number of bytes requested
  // from the network depends on the size of _buffer at the start
  // of the function and is tpically less than bytes.
  typedef std::function<void(const boost::system::error_code&, std::size_t)> StateMachineCallback;
  void readUpTo(size_t bytes,
                StateMachineCallback callback);

  // This function is "reset the receive state machine"
  void restartReceiveCycle();

  // All rx* functions are states in the receive state machine
  void rxFirstByteOculusId(const boost::system::error_code& ec,
                  std::size_t bytes_transferred);

  void rxSecondByteOculusId(const boost::system::error_code& ec,
                  std::size_t bytes_transferred);

  void rxHeader(const boost::system::error_code& ec,
                  std::size_t bytes_transferred);

  void rxSimplePingResult(const boost::system::error_code& ec,
                            std::size_t bytes_transferred);

  void rxIgnoredData(const boost::system::error_code& ec,
                            std::size_t bytes_transferred);

  void rxMessageLogs(const boost::system::error_code& ec,
                            std::size_t bytes_transferred);

  boost::asio::ip::tcp::socket _socket;

  //
  ByteVector _buffer;

  SimplePingCallback _simplePingCallback;
  OnConnectCallback _onConnectCallback;

};  // class DataRx
}  // namespace liboculus
