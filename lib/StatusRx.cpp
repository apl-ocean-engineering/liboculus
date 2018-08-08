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

#include <arpa/inet.h>

#include <boost/bind.hpp>

#include "liboculus/StatusRx.h"
#include "g3log/g3log.hpp"

namespace liboculus {

  using std::string;

  using boost::asio::ip::address_v4;

  SonarStatus::SonarStatus()
  : _writeLock(),
    _valid(false)
  {}


  OculusStatusMsg SonarStatus::operator()( void ) const
  {
    std::lock_guard<std::mutex> lock( _writeLock );

    // Copy constructor?
    OculusStatusMsg osm;
    memcpy( (void *)&osm, (void *)&_osm, sizeof(OculusStatusMsg) );

    return osm;
  }

  boost::asio::ip::address SonarStatus::ipAddr() const {
    std::lock_guard<std::mutex> lock( _writeLock );
    return address_v4( ntohl( _osm.ipAddr ));
  }

  void SonarStatus::update( const OculusStatusMsg &msg )
  {
    std::lock_guard<std::mutex> lock( _writeLock );

    memcpy( (void *)&_osm, (void *)&msg, sizeof(OculusStatusMsg) );
    _valid = true;
  }




    // ----------------------------------------------------------------------------
    // OsStatusRx - a listening socket for oculus status messages

    OsStatusRx::OsStatusRx(boost::asio::io_service &context, const std::shared_ptr<SonarStatus> &status )
    : _status( status ),
    _ioService(context),
    _socket(_ioService),
    _inputBuffer( sizeof(OculusStatusMsg) ),
    _deadline(_ioService)
    {
      // Create and setup a broadcast listening socket
      m_port     = 52102;   // fixed port for status messages
      m_valid    = 0;
      m_invalid  = 0;

      doConnect();
    }

    OsStatusRx::~OsStatusRx()
    {

    }

    void OsStatusRx::doConnect()
    {
      boost::asio::ip::udp::endpoint local(
        boost::asio::ip::address_v4::any(),
        52102);
        boost::system::error_code error;

        _socket.open(boost::asio::ip::udp::v4(), error);

        boost::asio::socket_base::broadcast option(true);
        _socket.set_option(option);

        if(!error) {
          _socket.bind(local);

          startReader();
        }
      }

      void OsStatusRx::startReader()
      {
        // Set a deadline for the read operation.
        //deadline_.expires_from_now(boost::posix_time::seconds(30));

        // Start an asynchronous receive
        _socket.async_receive( boost::asio::buffer((void *)&_osm, sizeof(OculusStatusMsg)),
        boost::bind(&OsStatusRx::handleRead, this, _1, _2));
      }

      void OsStatusRx::handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred )
      {
        // if (stopped_)
        //   return;

        if (!ec)
        {
          // Extract the newline-delimited message from the buffer.

          if( bytes_transferred != sizeof(OculusStatusMsg)) {
            LOG(WARNING) << "Got " << bytes_transferred << " bytes, expected OculusStatusMsg of size " << sizeof(OculusStatusMsg);
            return;
          }

          LOG(DEBUG) << "Device id " << _osm.deviceId << " ; type: " <<  (uint16_t)_osm.deviceType;

          LOG(DEBUG) << "      Sonar ip addr: " << boost::asio::ip::address_v4( ntohl(_osm.ipAddr) );
          LOG(DEBUG) << " Sonar connected to: " << boost::asio::ip::address_v4( ntohl(_osm.connectedIpAddr) );

          _status->update( _osm );
          m_valid++;

          // Schedule another read
          startReader();
        }
        else
        {
          LOG(WARNING) << "Error on receive: " << ec.message();

          //stop();
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
