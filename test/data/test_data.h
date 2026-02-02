// Copyright (c) 2017-2026 University of Washington
// Author: Aaron Marburg <amarburg@uw.edu>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of University of Washington nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <gtest/gtest.h>

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "liboculus/SimplePingResult.h"

// NOTE: must be set via CMake
#ifndef TEST_DATA_PATH
#error "TEST_DATA_PATH must be defined for unit test data to be found"
#endif

#define ONE_RAW_PING (TEST_DATA_PATH "/one_ping_8bit.raw")
#define THREE_RAW_PINGS (TEST_DATA_PATH "/three_pings_8bit.raw")

namespace Oculus_TestData {

using liboculus::ByteVector;
using std::vector;

inline std::shared_ptr<ByteVector> Load(const std::string &filename) {
  std::ifstream inf(filename);

  if (!inf.is_open())
    return std::make_shared<ByteVector>();

  // This feels a little ... wrong
  inf.seekg(0, std::ios::end);

  const size_t sz = inf.tellg();

  std::string out(sz, '\0');
  inf.seekg(0, std::ios::beg);
  inf.read(&out[0], sz);

  return std::make_shared<ByteVector>(out.begin(), out.end());
}

} // namespace Oculus_TestData
