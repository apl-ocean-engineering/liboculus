/******************************************************************************
* This file has been derived from the original Blueprint Subsea
* Oculus SDK file "StatusRx.h".
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

    // ----------------------------------------------------------------------------
    // StatusRx - a listening socket for oculus status messages

    StatusRx::StatusRx(boost::asio::io_service &context )
                : _status(),
                _ioService(context),
                _socket(_ioService),
                _inputBuffer( sizeof(OculusStatusMsg) ),
                _deadline(_ioService),
                _sonarStatusCallback( std::bind( &StatusRx::defaultSonarStatusCallback, this, std::placeholders::_1 ))
    {
      // Create and setup a broadcast listening socket
      m_port     = 52102;   // fixed port for status messages
      m_valid    = 0;
      m_invalid  = 0;

      doConnect();
    }

    StatusRx::~StatusRx()
    {

    }

    void StatusRx::doConnect()
    {
      boost::asio::ip::udp::endpoint local( boost::asio::ip::address_v4::any(), 52102);
        boost::system::error_code error;

        _socket.open(boost::asio::ip::udp::v4(), error);

        boost::asio::socket_base::broadcast option(true);
        _socket.set_option(option);

        if(!error) {
          _socket.bind(local);

          startReader();
        } else {
          LOG(WARNING) << "Unable to start reader";
        }
      }

      void StatusRx::startReader()
      {
        // Set a deadline for the read operation.
        //deadline_.expires_from_now(boost::posix_time::seconds(30));

        // Start an asynchronous receive
        _socket.async_receive( boost::asio::buffer((void *)&_osm, sizeof(OculusStatusMsg)), boost::bind(&StatusRx::handleRead, this, _1, _2));
      }

      void StatusRx::handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred )
      {
        if (!ec) {
          // Extract the newline-delimited message from the buffer.

          if( bytes_transferred != sizeof(OculusStatusMsg)) {
            LOG(WARNING) << "Got " << bytes_transferred << " bytes, expected OculusStatusMsg of size " << sizeof(OculusStatusMsg);
            return;
          }

          LOG(DEBUG) << "Got status message.  Updating!";
          _status.update( _osm );
          _sonarStatusCallback( _status );
          
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


    }
