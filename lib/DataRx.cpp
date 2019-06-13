/******************************************************************************
 * This file has been derived from the original Blueprint Subsea
 * Oculus SDK file "DataRx.h".
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
#include <chrono>

#include "liboculus/DataRx.h"

#include "g3log/g3log.hpp"

namespace liboculus {

using std::string;
using std::shared_ptr;

// ----------------------------------------------------------------------------
// DataRx - a listening socket for oculus status messages

DataRx::DataRx(boost::asio::io_service &context, uint32_t ip,
                          const SimpleFireMessage &fire )
  : _ioService(context),
    _ipAddress( boost::asio::ip::address_v4(ip) ),
    _socket(_ioService),
    _writeTimer(_ioService),
    _fireMessage(fire),
    _hdr(),
    _queue()
{
  doConnect();
}

DataRx::DataRx(boost::asio::io_service &context,
                          const boost::asio::ip::address &addr,
                          const SimpleFireMessage &fire )
  : _ioService(context),
    _ipAddress( addr ),
    _socket(_ioService),
    _writeTimer(_ioService),
    _fireMessage(fire),
    _hdr(),
    _queue()
{
  doConnect();
}

DataRx::~DataRx()
{
}

void DataRx::doConnect()
{
  uint16_t _port = 52100;

  boost::asio::ip::tcp::endpoint sonarEndpoint( _ipAddress, _port);

  LOG(DEBUG) << "Connecting to sonar at " << sonarEndpoint;

  _socket.async_connect( sonarEndpoint, boost::bind(&DataRx::onConnect, this, _1) );
}


void DataRx::onConnect(const boost::system::error_code& ec)
{
  if (!ec) {
    scheduleHeaderRead();

    // Send one SimpleFireMessage immediately.
    writeHandler( ec );

  } else {
    LOG(WARNING) << "Error on connect: " << ec.message();
  }
}

//== Data writers

// Schedule a writer in 500ms
void DataRx::scheduleWrite()
{
  _writeTimer.expires_from_now(std::chrono::milliseconds(1000));
  _writeTimer.async_wait(boost::bind(&DataRx::writeHandler, this, _1));
}

// Write _fireMessage
void DataRx::writeHandler(const boost::system::error_code& ec )
{
  if( !ec ) {
    boost::asio::streambuf msg;

    _fireMessage.serialize(msg);

    auto result = _socket.send( msg.data() );
    LOG(DEBUG) << "Sent " << result << " bytes to sonar";
  } else {
    LOG(WARNING) << "Error on write: " << ec.message();
  }

  // Schedule the next write
  //scheduleWrite();
}

void DataRx::updateFireMessage( const SimpleFireMessage &msg ) {
  _fireMessage = msg;

  boost::asio::streambuf buf;
  _fireMessage.serialize(buf);
  auto result = _socket.send( buf.data() );
  LOG(DEBUG) << "Sent " << result << " bytes to sonar";
}

//=== Readers
void DataRx::scheduleHeaderRead()
{
  _socket.async_receive( boost::asio::buffer((void *)&_hdr.hdr, sizeof(OculusMessageHeader)),
                         boost::bind(&DataRx::readHeader, this, _1, _2));
}


void DataRx::readHeader(const boost::system::error_code& ec, std::size_t bytes_transferred )
{
  if (!ec) {

    LOG(DEBUG) << "Got " << bytes_transferred << " bytes of header from sonar";

    if( bytes_transferred == sizeof(OculusMessageHeader) ) {

      LOG(DEBUG) << "Validating...";
      if( _hdr.valid() ) {

            if( _hdr.msgId() == messageSimplePingResult ) {

              // Rely on ref-counting of shared_ptr to clean up any dropped packets
              shared_ptr<SimplePingResult> ping( new SimplePingResult( _hdr ) );

              // Read the ping hedaer
              auto b = boost::asio::buffer( ping->ptrAfterHeader(), ping->hdr()->payloadSize );
              LOG(DEBUG) << "Requesting balance of SimplePingResult header, " << ping->hdr()->payloadSize << " bytes";

//_socket.async_read( b, boost::bind(&DataRx::readSimplePingResult, this, ping, _1, _2));
	      boost::asio::async_read( _socket, b, boost::bind(&DataRx::readSimplePingResult, this, ping, _1, _2));

            } else if ( _hdr.msgId() == messageLogs && _hdr.hdr.payloadSize > 0 ) {

                LOG(DEBUG) << "Requesting balance of Log message";

                boost::asio::streambuf junkBuffer(_hdr.hdr.payloadSize);

                auto bytes_recvd = boost::asio::read( _socket, junkBuffer );
                                        //
                                        // [this, junkBuffer](boost::system::error_code ec, std::size_t bytes_recvd)
                                        //         {

                LOG(DEBUG) << "Read " << bytes_recvd << " of logging info";
                if (bytes_recvd > 0)
                {
                  std::string s( (std::istreambuf_iterator<char>(&junkBuffer)), std::istreambuf_iterator<char>() );
                  LOG(DEBUG) << s;

                  scheduleHeaderRead();
                }
                else
                {
                  LOG(WARNING) << "Error on receive of add'l data: " << ec.message();
                }

            } else {
              // Drop the rest of the message

              const size_t discardSz = _hdr.hdr.payloadSize;
              LOG(INFO) << "Trying to drain an additional " << discardSz << " bytes";

              if( discardSz > 0 ) {
                std::vector<char> junkBuffer(_hdr.hdr.payloadSize);

                boost::asio::async_read( _socket, boost::asio::buffer( junkBuffer, discardSz),
                                        [this](boost::system::error_code ec, std::size_t bytes_recvd)
                                                {
                                                  LOG(DEBUG) << "Read and discarded " << bytes_recvd;
                                                  if (!ec && bytes_recvd > 0)
                                                  {
                                                    scheduleHeaderRead();
                                                  }
                                                  else
                                                  {
                                                    LOG(WARNING) << "Error on receive of add'l data: " << ec.message();
                                                  }

                                                  //delete junkBuffer;
                                                });

                } else {
                  // Otherwise, just schedule the next read
                  scheduleHeaderRead();
                }

              return;
            }

      } else {
        LOG(WARNING) << "Incoming header invalid";
      }

    } else {
      LOG(WARNING) << "Received short header of " << bytes_transferred << " expected " << sizeof(OculusMessageHeader);
    }

  } else {
    LOG(WARNING) << "Error on receive of header: " << ec.message();
  }
}

void DataRx::readSimplePingResult( const shared_ptr<SimplePingResult> &ping,
                                            const boost::system::error_code& ec,
                                            std::size_t bytes_transferred )
{
  if (!ec) {
    LOG(DEBUG) << "Got " << bytes_transferred << " bytes of SimplePingResult from sonar";

    if( bytes_transferred == ping->hdr()->payloadSize ) {

      if( ping->update() ) {

        LOG(DEBUG) << "Data valid!";

        _queue.push( ping );

        // And return to the home state
        scheduleHeaderRead();
      } else {
        LOG(WARNING) << "Incoming packet invalid";
      }


    } else {
      LOG(WARNING) << "Received short header of " << bytes_transferred << " expected " << ping->hdr()->payloadSize;
    }

  } else {
    LOG(WARNING) << "Error on receive of header: " << ec.message();
  }
}

}
