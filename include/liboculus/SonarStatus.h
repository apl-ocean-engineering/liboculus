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

 #pragma once

#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

#include <boost/asio.hpp>

#include "g3log/g3log.hpp"

#include "Oculus/Oculus.h"

namespace liboculus {


class SonarStatus
{
public:

  typedef std::chrono::time_point<std::chrono::system_clock> sys_time_point;

  SonarStatus();

  OculusStatusMsg operator()( void ) const;

  bool wait() const
  {
    std::unique_lock<std::mutex> lock(_statusMutex);
    _statusUpdateCond.wait(lock);

    return true;
  }

  template< class Rep, class Period = std::ratio<1> >
  bool wait_for( const std::chrono::duration<Rep,Period> &timeout ) const
  {
    std::unique_lock<std::mutex> lock(_statusMutex);
    return (_statusUpdateCond.wait_for( lock, timeout ) != std::cv_status::timeout);
  }

  bool valid() const { return _valid; }
  void dump() const;

  boost::asio::ip::address ipAddr() const;

  void update( const OculusStatusMsg &msg, sys_time_point msgTime = std::chrono::system_clock::now() );

protected:

  mutable std::mutex       _statusMutex;
  mutable std::condition_variable  _statusUpdateCond;

private:

  bool                  _valid;
  OculusStatusMsg       _osm;             // The more recent status message
  sys_time_point        _msgTime;     // The time of the last message
};

}
