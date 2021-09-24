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

#include "SimplePingResult.h"
#include "SonarConfiguration.h"

#include <memory>

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>

#include "Oculus/Oculus.h"


namespace liboculus {

using std::shared_ptr;
using boost::asio::ip::tcp;

// ----------------------------------------------------------------------------
// DataRx - a state machine for receiving sonar data over the network

class DataRx {
 public:
  // n.b. takes IP in __NETWORK__ byte order
  explicit DataRx(boost::asio::io_service &context);

  // NB: The config is NOT const. This is how the driver's configuration
  //     is hooked up to the instrument (so DataRx to call setCallback)
  // TODO(lindzey): This architecture is very ugly. Consider rewriting
  //     so the config's callback is a function in the client, who in
  //     turn directly calls the function in DataRx, which it owns.
  DataRx(boost::asio::io_service &context, uint32_t ip,
         SonarConfiguration &config);

  DataRx(boost::asio::io_service &context,
         const boost::asio::ip::address &addr,
         SonarConfiguration &config);

  virtual ~DataRx() {}

  void connect(uint32_t ip, SonarConfiguration &config);
  void connect(const boost::asio::ip::address &addr,
               SonarConfiguration &config);

  bool connected() const { return _socket.is_open(); }

  typedef std::function< void(const SimplePingResult &) > SimplePingCallback;
  void setCallback(SimplePingCallback callback);

 private:
  // Callback for when the socket is connected. Schedule the first header read
  // and send the first configuration to the sensor.
  void onConnect(const boost::system::error_code& error,
                 const SonarConfiguration &config);

  // Request bytes from the socket, set up readHeader as callback
  void scheduleHeaderRead();
  // Callback for when header bytes have been received.
  // NOTE(lindzey): Given how much trouble the rest of this driver goes to
  //   to avoid copying data, it seems odd that the MessageHeaders are being
  //   passed around by value.
  void readHeader(MessageHeader hdr,
                  const boost::system::error_code& ec,
                  std::size_t bytes_transferred);
  // Callback for when payload bytes have been received for a message known
  // to be a simplePingResult. Stuff them into a SimplePingResult and pass
  // it along to the registered callback.
  void readSimplePingResult(MessageHeader hdr,
                            const boost::system::error_code& ec,
                            std::size_t bytes_transferred);

  // Immediately send configuration update to the sonar
  void sendConfiguration(const SonarConfiguration &config);

  boost::asio::io_service  &_ioService;
  tcp::socket _socket;

  SimplePingCallback _simplePingCallback;
};

}  // namespace liboculus
