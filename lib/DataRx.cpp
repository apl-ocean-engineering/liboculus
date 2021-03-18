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

  DataRx::DataRx(boost::asio::io_service &context )
    : _ioService(context),
      _socket(_ioService),
      _simplePingCallback()
    {
      ;
    }

  DataRx::DataRx(boost::asio::io_service &context, uint32_t ip,  SonarConfiguration &config )
    : _ioService(context),
      _socket(_ioService),
      _simplePingCallback()
    {
      connect( boost::asio::ip::address_v4( ip ), config);
    }

  DataRx::DataRx(boost::asio::io_service &context, const boost::asio::ip::address &addr,  SonarConfiguration &config )
    : _ioService(context),
      _socket(_ioService),
      _simplePingCallback()
  {
    connect( addr, config );
  }

  DataRx::~DataRx()
  {
  }

  void DataRx::setCallback( SimplePingCallback callback ) {
    _simplePingCallback = callback;
  }

  void DataRx::connect( uint32_t ip,  SonarConfiguration &config ) {
    connect(boost::asio::ip::address_v4( ip ), config);
  }

  void DataRx::connect(const boost::asio::ip::address &addr,  SonarConfiguration &config)
  {
    if( connected() ) return;

    uint16_t port = 52100;
    boost::asio::ip::address ipAddress = addr;

    boost::asio::ip::tcp::endpoint sonarEndpoint( ipAddress, port);

    LOG(DEBUG) << "Connecting to sonar at " << sonarEndpoint;

    config.setCallback( std::bind( &DataRx::sendConfiguration, this, std::placeholders::_1 ));

    // Schedule the first async_io connect task
    _socket.async_connect( sonarEndpoint, boost::bind(&DataRx::onConnect, this, _1, config ) );
  }


  void DataRx::onConnect(const boost::system::error_code& ec, SonarConfiguration &config ) {
    if (!ec) {
      scheduleHeaderRead();

      // Send one SonarConfiguration immediately.
      sendConfiguration(config);

    } else {
      LOG(WARNING) << "Error on connect: " << ec.message();
    }
  }

  //== Data writers

  void DataRx::sendConfiguration( const SonarConfiguration &msg ) {
    // Send it out immediately
    boost::asio::streambuf buf;
    msg.serializeTo( buf );

    auto result = _socket.send(  buf.data() );
    LOG(DEBUG) << "Sent " << result << " bytes to sonar";
  }

  //=== Readers
  void DataRx::scheduleHeaderRead() {
    MessageHeader header;

    _socket.async_receive( boost::asio::buffer((void *)header.ptr(), sizeof(OculusMessageHeader)),
                           boost::bind(&DataRx::readHeader, this, header, _1, _2));
  }


  void DataRx::readHeader( MessageHeader hdr, const boost::system::error_code& ec, std::size_t bytes_transferred )
  {
    if (!ec) {

      LOG(DEBUG) << "Got " << bytes_transferred << " bytes of header from sonar";

      if( bytes_transferred == sizeof(OculusMessageHeader) ) {

        LOG(DEBUG) << "Validating...";
        if( hdr.valid() ) {

          LOG(DEBUG) << "Got message ID " << static_cast<int>(hdr.msgId());
          if( hdr.msgId() == messageSimplePingResult ) {

            if( !hdr.expandForPayload() ) {
              LOG(WARNING) << "Unable to expand for payload";
            }

            // Rely on ref-counting of shared_ptr to clean up any dropped packets
            //shared_ptr<SimplePingResult> ping( new SimplePingResult( _hdr ) );

            // Read the remainder of the packet
            auto b = boost::asio::buffer( hdr.payloadPtr(), hdr.alignedPayloadSize() );
            boost::asio::async_read( _socket, b, boost::bind(&DataRx::readSimplePingResult, this, hdr, _1, _2));

          } else if ( hdr.msgId() == messageLogs && hdr.payloadSize() > 0 ) {

            LOG(DEBUG) << "Requesting balance of Log message";

            boost::asio::streambuf junkBuffer( hdr.payloadSize() );
            auto bytes_recvd = boost::asio::read( _socket, junkBuffer );

            LOG(DEBUG) << "Read " << bytes_recvd << " of logging info";
            if (bytes_recvd > 0)
            {
              std::string s( (std::istreambuf_iterator<char>(&junkBuffer)), std::istreambuf_iterator<char>() );
              LOG(DEBUG) << s;

              scheduleHeaderRead();
            }
            else
            {
              LOG(WARNING) << "Error on receive of payload for log message: " << ec.message();
            }

          } else {
            // Drop the rest of the message

            const size_t discardSz = hdr.alignedPayloadSize();

            if ( discardSz == 0 ) {
              if (hdr.msgId() == messageDummy ) {
                LOG(DEBUG) << "Ignoring dummy message";
              } else {
                LOG(INFO) << "Unknown message ID " << static_cast<int>(hdr.msgId());
              }
              scheduleHeaderRead();
            } else {
              LOG(INFO) << "Unknown message ID " << static_cast<int>(hdr.msgId()) << ", need to drain an additional " << discardSz << " bytes";

              std::vector<char> junkBuffer(discardSz);

              boost::asio::async_read( _socket, boost::asio::buffer( junkBuffer, discardSz),
                  [this](boost::system::error_code ec, std::size_t bytes_recvd)
                  {
                    LOG(DEBUG) << "Read and discarded " << bytes_recvd;
                    if (!ec && bytes_recvd > 0) {
                      scheduleHeaderRead();
                    } else {
                      LOG(WARNING) << "Error on receive of add'l data: " << ec.message();
                    }

                    //delete junkBuffer;
                  });

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

  void DataRx::readSimplePingResult(  MessageHeader hdr,
                                      const boost::system::error_code& ec,
                                      std::size_t bytes_transferred )
  {
      if (!ec) {
        LOG(DEBUG) << "Got " << bytes_transferred << " bytes of SimplePingResult from sonar";

        if( bytes_transferred == hdr.alignedPayloadSize() ) {

          SimplePingResult ping( hdr );

          if( ping.valid() ) {

            LOG(DEBUG) << "Data valid!";

            _simplePingCallback( ping );

            // And return to the home state
            scheduleHeaderRead();
          } else {
            LOG(WARNING) << "Incoming packet invalid";
          }


        } else {
          LOG(WARNING) << "Received short header of " << bytes_transferred << " expected " << hdr.payloadSize();
        }

      } else {
        LOG(WARNING) << "Error on receive of header: " << ec.message();
      }
    }


}
