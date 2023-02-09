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

#include <netinet/in.h>

#include <g3log/g3log.hpp>  // needed for CHECK macro
#include <iostream>
#include <vector>

#include "Oculus/Oculus.h"
#include "liboculus/DataTypes.h"

namespace liboculus {

// \todo  A long-term TODO: ImageData, GainData and BearingData are all
// fairly similar in functionality, could reduce the DRY?
template <typename T>
class GainData {
 public:
  typedef T DataType;

  GainData() : _data() { ; }

  GainData(const GainData &other) = default;

  GainData(const T *data, uint32_t imageSz, size_t strideBytes, size_t nRanges)
      : _data() {
    // Only works for four-byte types
    assert(sizeof(T) == 4);
    for (size_t i = 0; i < nRanges; i++) {
      const size_t index = (i * strideBytes) / sizeof(T);

      const uint32_t *d = reinterpret_cast<const uint32_t *>(&data[index]);

      // uint32_t h = ntohl(*d);
      uint32_t h = *d;
      T *f = reinterpret_cast<T *>(&h);

      _data.push_back(*f);
    }
  }

  int size() const { return _data.size(); }

  T at(unsigned int i) const { return _data.at(i); }

  T operator[](unsigned int i) const { return at(i); }

 private:
  std::vector<T> _data;
};

}  // namespace liboculus
