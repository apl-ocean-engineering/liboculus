/******************************************************************************
 * This file has been derived from the original Blueprint Subsea
 * Oculus SDK file "StatusRx.h".
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

#include <mutex>
#include <chrono>

#include "Oculus/Oculus.h"

#include <boost/asio.hpp>

#include "liboculus/SonarStatus.h"


namespace liboculus {

  using boost::asio::ip::udp;
  using boost::asio::deadline_timer;

// ----------------------------------------------------------------------------
// StatusRx - a listening socket for oculus status messages
//
//
class StatusRx
{
public:
    StatusRx(boost::asio::io_service &context );
    ~StatusRx();

    const SonarStatus &status() const
      { return _status; }

    typedef std::function< void( const SonarStatus & ) > SonarStatusCallback;

    void setCallback( SonarStatusCallback callback )
      { _sonarStatusCallback = callback; }


private:

  void doConnect();

  void startReader();
  void handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred );

//  void doReceiveStatusMessage();

  // "Scratch" copy, network operations write directly to this copy
  OculusStatusMsg _osm;

  // This is the "usable" status, based on the most recently received packet
  SonarStatus _status;

  uint16_t     m_port;       // Port to listen on
  uint16_t     m_valid;      // Number of valid status messages
  uint16_t     m_invalid;    // Number of invalid status messages

  boost::asio::io_service& _ioService;
  udp::socket _socket;

  boost::asio::streambuf _inputBuffer;
  deadline_timer _deadline;

  SonarStatusCallback _sonarStatusCallback;
  void defaultSonarStatusCallback( const SonarStatus & ) {;}
};

}
