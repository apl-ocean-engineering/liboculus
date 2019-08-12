/******************************************************************************
 * This file has been derived from the original Blueprint Subsea
 * Oculus SDK file "OsStatusRx.h".
 *
 * The original Oculus copyright notice follows
 *
 * (c) Copyright 2017 Blueprint Subsea.
 * This file is part of Oculus Viewer
 *
 * Oculus Viewer is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Oculus Viewer is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/

#pragma once

#include <memory>

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>

#include "active_object/bounded_shared_queue.h"

#include "Oculus/Oculus.h"

#include "SimplePingResult.h"
#include "SimpleFireMessage.h"

namespace liboculus {

  using std::shared_ptr;
  using boost::asio::ip::tcp;

// ----------------------------------------------------------------------------
// DataRx - a state machine for receiving sonar data over the network

// TODO.   Currently, the queue depth is fixed at 20.   Template?

class DataRx
{
public:

  // n.b. takes IP in __NETWORK__ byte order
  DataRx(boost::asio::io_service &context, const SimpleFireMessage &fire = SimpleFireMessage() );

  DataRx(boost::asio::io_service &context, uint32_t ip,
              const SimpleFireMessage &fire = SimpleFireMessage() );

  DataRx(boost::asio::io_service &context, const boost::asio::ip::address &addr,
              const SimpleFireMessage &fire = SimpleFireMessage() );

  virtual ~DataRx();

  void connect( uint32_t ip );
  void connect( const boost::asio::ip::address &addr );

  bool connected() const { return _socket.is_open(); }

  SimpleFireMessage fireMessage() { return _fireMessage; }
  void updateFireMessage( const SimpleFireMessage &msg );

  typedef std::function< void( const shared_ptr<SimplePingResult> & ) > SimplePingCallback;
  void setCallback( SimplePingCallback callback );

private:


  void onConnect(const boost::system::error_code& error);

  void scheduleWrite();
  void writeHandler(const boost::system::error_code& ec );

  void scheduleHeaderRead();
  void readHeader( const shared_ptr<MessageBuffer> &buffer, const boost::system::error_code& ec, std::size_t bytes_transferred );
  void readSimplePingResult( const shared_ptr<MessageBuffer> &buffer, const boost::system::error_code& ec, std::size_t bytes_transferred );


  boost::asio::io_service  &_ioService;
  tcp::socket _socket;

  boost::asio::steady_timer _writeTimer;

  // Configuration data out to sonar
  SimpleFireMessage _fireMessage;

  SimplePingCallback _simplePingCallback;

  void defaultSimplePingCallback(  const shared_ptr<SimplePingResult> & ) {;}

};

class DataRxQueued : public DataRx {
public:
  typedef active_object::bounded_shared_queue< shared_ptr<SimplePingResult>, 20 > Queue;

  DataRxQueued(boost::asio::io_service &context, uint32_t ip,
              const SimpleFireMessage &fire = SimpleFireMessage() );

  DataRxQueued(boost::asio::io_service &context, const boost::asio::ip::address &addr,
              const SimpleFireMessage &fire = SimpleFireMessage() );

  virtual ~DataRxQueued();

  Queue &queue() { return _queue; }

  void enqueuePing( const shared_ptr<SimplePingResult> &ping );

private:

  Queue _queue;

};


}
