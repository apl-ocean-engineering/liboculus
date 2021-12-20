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

#include <boost/bind.hpp>

#include "liboculus/IoServiceThread.h"

namespace liboculus {

  IoServiceThread::IoServiceThread()
      : _context(std::make_shared<boost::asio::io_context>()),
        _work_guard(_context->get_executor()),
        _thread() {}

  IoServiceThread::~IoServiceThread() {
  }

  void IoServiceThread::start() {
    if (_thread) return; // running
    _thread.reset(new std::thread(boost::bind(&IoServiceThread::threadExec,
                                               this)));
  }

  void IoServiceThread::stop() {
    if (!_thread) return;
    _work_guard.reset();
    _context->stop();
  }

  void IoServiceThread::join() {
    if (!_thread) return;
    _thread->join();
    _context->reset();
    _thread.reset();
  }

  void IoServiceThread::threadExec() {
    _context->run();
  }


}  // namespace liboculus