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
// Operates in a kind of double-buffering mode.   As new packets comes in,
// they are assembled in _osm.   Once received and validated, it gets
// copied to _status->osm
//
// Two modes of operation:
//    If the constructor is provided with a shared_ptr<SonarStatus>,
//    will copy the latest status packet into that struct.
//
//    If not provided, it will make its own internally, which can be
//    queried with status()
//
class StatusRx
{
public:
    StatusRx(boost::asio::io_service &context,
                const std::shared_ptr<SonarStatus> &status = std::shared_ptr<SonarStatus>(new SonarStatus()) );
    ~StatusRx();

    const SonarStatus &status() const { return *_status; }



private:

  void doConnect();

  void startReader();
  void handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred );

//  void doReceiveStatusMessage();

  // "Scratch" copy, network operations write directly to this copy
  OculusStatusMsg _osm;

  // This is the "usable" status, based on the most recently received packet
  std::shared_ptr<SonarStatus> _status;

  uint16_t     m_port;       // Port to listen on
  uint16_t     m_valid;      // Number of valid status messages
  uint16_t     m_invalid;    // Number of invalid status messages

  boost::asio::io_service& _ioService;
  udp::socket _socket;

  boost::asio::streambuf _inputBuffer;
  deadline_timer _deadline;
};

}
