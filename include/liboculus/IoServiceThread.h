/*
 * Copyright (c) 2017-2020 University of Washingon
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
#include <boost/bind.hpp>

#include "g3log/g3log.hpp"

namespace liboculus {

// Generic "worker thread" for boost::asio
class IoServiceThread {
 public:
  IoServiceThread()
      : _context(),
        _work_guard(make_work_guard(_context)),
        _thread() {}

  ~IoServiceThread() {
    LOG(WARNING) << "Destructor";
  }

  void start() {
    if (_thread) return; // running

    LOG(WARNING) << "Starting IoServiceThread";
    // _thread.reset(new std::thread(boost::bind(&boost::asio::io_context::run,
    //                                           &_context)));
    _thread.reset(new std::thread(boost::bind(&IoServiceThread::threadExec,
                                               this)));
  }

  void stop() {
    if (!_thread) return;
    LOG(WARNING) << "Stopping IoServiceThread";
    _work_guard.reset();
    _context.stop();
  }

  void join() {
    if (!_thread) return;
    _thread->join();
    _context.reset();
    _thread.reset();
  }

  void threadExec() {
    LOG(WARNING) << "threadExec begin";
    for (;;) {
      try {
        _context.restart();
        _context.run();
        LOG(WARNING) << "Context " << (_context.stopped() ? "is" : "is not") << " stopped";
        break; // run() exited normally
      } catch (...) {
        LOG(WARNING) << "Exception in io_context";
      }
    }
    LOG(WARNING) << "threadExec end";
  }

  boost::asio::io_context &context() { return _context; }

 private:
  boost::asio::io_context _context;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> _work_guard;
  
  std::unique_ptr<std::thread> _thread;
};

}
