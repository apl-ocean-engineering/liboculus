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

#include <memory>

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>

#include "active_object/bounded_shared_queue.h"

#include "Oculus/Oculus.h"

#include "SimplePingResult.h"
#include "SonarConfiguration.h"

namespace liboculus {

  using std::shared_ptr;
  using boost::asio::ip::tcp;

// ----------------------------------------------------------------------------
// DataRx - a state machine for receiving sonar data over the network

class DataRx {
public:

  // n.b. takes IP in __NETWORK__ byte order
  DataRx(boost::asio::io_service &context );

  DataRx(boost::asio::io_service &context, uint32_t ip,
               SonarConfiguration &config );

  DataRx(boost::asio::io_service &context, const boost::asio::ip::address &addr,
               SonarConfiguration &config );

  virtual ~DataRx();

  void connect( uint32_t ip,  SonarConfiguration &config );
  void connect( const boost::asio::ip::address &addr,  SonarConfiguration &config );

  bool connected() const { return _socket.is_open(); }

  typedef std::function< void( const SimplePingResult & ) > SimplePingCallback;
  void setCallback( SimplePingCallback callback );
private:


  void onConnect(const boost::system::error_code& error, SonarConfiguration &config);

  // void scheduleWrite();
  // void writeHandler(const boost::system::error_code& ec );

  void scheduleHeaderRead();
  void readHeader( MessageHeader hdr, const boost::system::error_code& ec, std::size_t bytes_transferred );
  void readSimplePingResult( MessageHeader hdr, const boost::system::error_code& ec, std::size_t bytes_transferred );

  void onSonarConfigurationChanged( const SonarConfiguration &config );

  boost::asio::io_service  &_ioService;
  tcp::socket _socket;

//  boost::asio::steady_timer _writeTimer;

  SimplePingCallback _simplePingCallback;

};

//===================================================================
// This is the old API which relied on a shared queue rather than a callback
// class DataRxQueued : public DataRx {
// public:
//   typedef active_object::bounded_shared_queue< shared_ptr<SimplePingResult>, 20 > Queue;
//
//   DataRxQueued(boost::asio::io_service &context, uint32_t ip,
//               const SonarConfiguration &fire = SonarConfiguration() );
//
//   DataRxQueued(boost::asio::io_service &context, const boost::asio::ip::address &addr,
//               const SonarConfiguration &fire = SonarConfiguration() );
//
//   virtual ~DataRxQueued();
//
//   Queue &queue() { return _queue; }
//
//   void enqueuePing( const shared_ptr<SimplePingResult> &ping );
//
// private:
//
//   Queue _queue;
//
// };


}
