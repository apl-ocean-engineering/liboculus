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

#include <fstream>
#include <memory>

#include "liboculus/SimplePingResult.h"
#include "liboculus/expected.hpp"

namespace liboculus {

class SonarPlayerBase {
public:
  typedef tl::expected<SimplePingResult<PingV1>, bool> SonarPlayerResult_t;

  SonarPlayerBase()           {;}
  virtual ~SonarPlayerBase()  {;}

  virtual bool open(const std::string &filename);
  virtual bool isOpen() const
    { return _input.is_open(); }

  virtual bool eof() const
    { return _input.eof(); }

  virtual void rewind()
    { _input.seekg(0); }

  virtual SonarPlayerResult_t nextPing() = 0;

  static std::shared_ptr<SonarPlayerBase> OpenFile(const std::string &filename);

// #ifdef WITH_GPMF
//   virtual bool setStream(GPMF_stream *stream) { return false; }
//
//   static std::shared_ptr<SonarPlayerBase> createGPMFSonarPlayer();
// #endif

protected:
  std::ifstream _input;
};

///
///
///
class RawSonarPlayer : public SonarPlayerBase {
 public:
  RawSonarPlayer()
    : SonarPlayerBase() {;}

  virtual ~RawSonarPlayer()
    {;}

  SonarPlayerBase::SonarPlayerResult_t nextPing() override;

 private:
};


} // namespace liboculus
