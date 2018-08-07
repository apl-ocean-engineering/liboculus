/******************************************************************************
 * This file has been derived from the original Blueprint Subsea
 * Oculus SDK file "OsStatusRx.h".
 *
 * The original Oculus copyright notie follows
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

#include <string.h>
#include <sstream>

#include <boost/bind.hpp>


#include "liboculus/StatusRx.h"
#include "g3log/g3log.hpp"


namespace liboculus {

using std::string;

// ----------------------------------------------------------------------------
// OsStatusRx - a listening socket for oculus status messages

OsStatusRx::OsStatusRx(boost::asio::io_service& context)
  : _ioService(context),
    _socket(_ioService)
{
  // Create and setup a broadcast listening socket
  //m_listener = new QUdpSocket(this);
  m_port     = 52102;   // fixed port for status messages
  m_valid    = 0;
  m_invalid  = 0;

  LOG(INFO) << "Connecting to status socket";

  string portStr;
  {
    std::ostringstream portS( portStr );
    portS << m_port;
  }

  udp::resolver resolver(_ioService);
  auto endpoints = resolver.resolve(udp::resolver::query(string("0.0.0.0"), portStr));

  doConnect(endpoints);

  // Connect the data signal
  //connect(m_listener, &QUdpSocket::readyRead, this, &OsStatusRx::ReadDatagrams);

  // Bind the socket (added Reuse address hint as this seems to allow other instances
  // of Oculus Viewer to see Sonars)

  //m_listener->bind(m_port, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
}

OsStatusRx::~OsStatusRx()
{

}

void OsStatusRx::doConnect(const udp::resolver::iterator& endpoints)
{
  boost::asio::async_connect(_socket, endpoints,
                            boost::bind(&OsStatusRx::handleConnect,
                                          this, _1, _2));
}

void OsStatusRx::handleConnect( const boost::system::error_code &ec, udp::resolver::iterator i )
{
  // Connect handler
  if (!ec) {
    LOG(INFO) << "Connected!";
    // On connector
  }
}

// void OsStatusRx::doReceiveStatusMessage()
// {
//   boost::asio::async_read(_socket,
//       boost::asio::buffer(read_msg_.data(), sizeof(OculusStatusMsg)),
//       [this](boost::system::error_code ec, std::size_t /*length*/)
//       {
//
//         if (!ec) {
//           read
//         } else {
//           _socket.close();
//         }
//       });
// }

// ----------------------------------------------------------------------------
// Signalled when there is data available in the socket buffer
// Note that if the Oculus Viewer software is running on a PC with two network ports then it is
// possible that both these ports will receive the status message
// In this case we will see twice as many status messages as expected
// void OsStatusRx::ReadDatagrams()
// {
//   // Read through any available datagrams
//   while (m_listener->hasPendingDatagrams())
//   {
//     // Read the datagram out of the socket buffer
//     QByteArray datagram;
//
//     datagram.resize(m_listener->pendingDatagramSize());
//     m_listener->readDatagram(datagram.data(), datagram.size());
//
//     // If datagra is of correct size, cast and signal any observers
//     if (datagram.size() == sizeof(OculusStatusMsg))
//     {
//       OculusStatusMsg osm;
//       memcpy(&osm, datagram.data(), datagram.size());
//
//       if (osm.hdr.oculusId == OCULUS_CHECK_ID)
//       {
//         m_valid++;
//
//         emit NewStatusMsg(osm, m_valid, m_invalid);
//       }
//     }
//     else
//     {
//       m_invalid++;
//     }
//   }
// }

}
