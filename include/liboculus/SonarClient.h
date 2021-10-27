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

#include <string>
#include <thread>
#include "liboculus/IoServiceThread.h"

#include "liboculus/StatusRx.h"
#include "liboculus/DataRx.h"

namespace liboculus {

class SonarClient {
 public:
  SonarClient(SonarConfiguration &config, const std::string &ipAddr = "");

  ~SonarClient();

  void start();
  void join();
  void stop();

  void updateConfiguration(const SonarConfiguration &msg);
  const SonarConfiguration &configuration() const;

  // Simple passthrough
  void setDataRxCallback(DataRx::SimplePingCallback callback) {
    _dataRx.setCallback(callback);
  }

 protected:
  void receiveStatus(const SonarStatus& status);

 private:
  std::string _ipAddr;

  IoServiceThread _ioSrv;
  // Status and Data messages come in on different ports, so they're
  // handled separately.
  StatusRx _statusRx;
  DataRx _dataRx;

  // unfortunately, need to have a copy when receiveStatus is called...
  // (We want the driver to be able to connect to the instrument whenever
  // it becomes available, which requires the SonarClient to know the
  // most recent configuration state, not just the state at startup.
  // Additionally, when _config is passed to DataRx, the configuration's
  // callback is bound to the proper member function for DataRx to update
  // the instrument's configuration.)
  SonarConfiguration &_config;

};  // class SonarClient
}  // namespace liboculus
