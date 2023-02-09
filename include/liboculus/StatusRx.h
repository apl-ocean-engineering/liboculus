/*
 * Copyright (c) 2017-2022 University of Washington
 * Author: Aaron Marburg <amarburg@uw.edu>
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

#pragma once

#include <boost/asio.hpp>
#include <chrono>
#include <memory>
#include <mutex>

#include "Oculus/Oculus.h"
#include "liboculus/IoServiceThread.h"
#include "liboculus/SonarStatus.h"

namespace liboculus {

using boost::asio::deadline_timer;
using boost::asio::ip::udp;

// ----------------------------------------------------------------------------
// StatusRx - a listening socket for oculus status messages
//
//
class StatusRx {
 public:
  explicit StatusRx(const IoServiceThread::IoContextPtr &iosrv);

  ~StatusRx() {}

  typedef std::function<void(const SonarStatus &, bool)> SonarStatusCallback;

  void setCallback(SonarStatusCallback callback) {
    _sonarStatusCallback = callback;
  }

 private:
  void doConnect();

  void scheduleRead();
  void handleRead(const boost::system::error_code &ec,
                  std::size_t bytes_transferred);

  bool parseStatus(const SonarStatus &status);

  std::vector<uint8_t> _buffer;

  // uint16_t     _port;       // Port to listen on
  uint16_t _num_valid_rx;    // Number of valid status messages
  uint16_t _num_invalid_rx;  // Number of invalid status messages

  udp::socket _socket;

  deadline_timer _deadline;

  SonarStatusCallback _sonarStatusCallback;
};

}  // namespace liboculus
