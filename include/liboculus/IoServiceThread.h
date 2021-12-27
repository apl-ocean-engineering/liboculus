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

#include <memory>
#include <thread>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "g3log/g3log.hpp"

namespace liboculus {

#if BOOST_VERSION >= 106600
using boost::asio::io_context;
#else
using boost::asio::io_service;
#endif

// Generic "worker thread" for boost::asio
class IoServiceThread {
 public:
  IoServiceThread();

  ~IoServiceThread();

  void start();

  void stop();
  void join();

#if BOOST_VERSION >= 106600
  const std::shared_ptr<io_context> &context() { return _context; }
#else
  const std::shared_ptr<io_service> &context() { return _context; }
#endif

 private:
#if BOOST_VERSION >= 106600
  std::shared_ptr<io_context> _context;

  // This class was added in later version of Boost;  not present in 1.65,
  // the version currently installed for 18.04 / ROS Melodic
  using work_guard_type = boost::asio::executor_work_guard<io_context::executor_type>;
  work_guard_type _work_guard;
#else
  std::shared_ptr<io_service> _context;
#endif

  std::unique_ptr<std::thread> _thread;

  void threadExec();
};

}
