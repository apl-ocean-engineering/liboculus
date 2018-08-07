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

#include "Oculus/Oculus.h"

#include <boost/asio.hpp>

using boost::asio::ip::udp;

namespace liboculus {

// ----------------------------------------------------------------------------
// Stores the last status message of a sonar witha given id
class COsStatusSonar
{
public:

  COsStatusSonar();

  unsigned        m_id;              // The id of the sonar
  OculusStatusMsg m_osm;             // The last status message associated with this sonar
  // QDateTime       m_lastMsgTime;     // The time of the last message
};


// ----------------------------------------------------------------------------
// OsStatusRx - a listening socket for oculus status messages

class OsStatusRx
{
public:
    OsStatusRx(boost::asio::io_service &context);
    ~OsStatusRx();

    void ReadDatagrams();

// signals:
//     void NewStatusMsg(OculusStatusMsg osm, uint16_6 valid, uint16_t invalid);

private:

  void doConnect(const udp::resolver::iterator &endpoints);

  void handleConnect( const boost::system::error_code& error, udp::resolver::iterator iterator );

//  void doReceiveStatusMessage();

  uint16_t     m_port;       // Port to listen on
  uint16_t     m_valid;      // Number of valid status messages
  uint16_t     m_invalid;    // Number of invalid status messages

  boost::asio::io_service& _ioService;
  udp::socket _socket;
};

}
