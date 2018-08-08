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

#include <boost/asio.hpp>
#include <boost/asio/steady_timer.hpp>

#include "Oculus/Oculus.h"

#include "SimpleFireMessage.h"

namespace liboculus {

  using boost::asio::ip::udp;

// ----------------------------------------------------------------------------
// OsStatusRx - a listening socket for oculus status messages

class SonarClient
{
public:
    SonarClient(boost::asio::io_service &context, uint32_t ip,
                const std::shared_ptr<SimpleFireMessage> &fire = std::shared_ptr<SimpleFireMessage>() );

    SonarClient(boost::asio::io_service &context, const boost::asio::ip::address &addr,
                const std::shared_ptr<SimpleFireMessage> &fire = std::shared_ptr<SimpleFireMessage>() );


    ~SonarClient();

private:

  void doConnect();

  void connectHandler(const boost::system::error_code& error);

  void scheduleWrite();
  void writeHandler(const boost::system::error_code& ec );

  void startReader();
  void readHeader(const boost::system::error_code& ec, std::size_t bytes_transferred );

  boost::asio::ip::address  _ipAddress;

  boost::asio::io_service& _ioService;
  udp::socket _socket;

  boost::asio::steady_timer _writeTimer;

  // Configuration data out to sonar
  std::shared_ptr<SimpleFireMessage> _fireMessage;

  // Data back from sonar
  OculusSimplePingResult _pingHeader;
};

}
