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
  explicit DataRx(boost::asio::io_context &iosrv);

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
  // rather than callbacks
  // The override *must* call this function for the data to be written
  // to the network
  virtual void haveWritten(const std::vector<uint8_t> &bytes) {;}
  virtual void haveRead(const std::vector<uint8_t> &bytes) {;}

 private:
  void onConnect(const boost::system::error_code& error);
 
  // This function is essentially "reset the state machine"
  void scheduleHeaderRead();


  // Callback for when header bytes have been received.
  // NOTE(lindzey): Given how much trouble the rest of this driver goes to
  //   to avoid copying data, it seems odd that the MessageHeaders are being
  //   passed around by value.
  void rxOculusId(const boost::system::error_code& ec,
                  std::size_t bytes_transferred);

  void rxHeader(const boost::system::error_code& ec,
                  std::size_t bytes_transferred);

  // Callback for when payload bytes have been received for a message known
  // to be a simplePingResult. Stuff them into a SimplePingResult and pass
  // it along to the registered callback.
  void readSimplePingResult(const boost::system::error_code& ec,
                            std::size_t bytes_transferred);

  // We got severe linkage issues if the IoServiceThread wasn't
  // instantiated within an object in the liboculus shared library.  
  // (shrug)
  boost::asio::ip::tcp::socket _socket;

  //
  ByteVector _buffer;

  SimplePingCallback _simplePingCallback;
  OnConnectCallback _onConnectCallback;

};  // class DataRx
}  // namespace liboculus
