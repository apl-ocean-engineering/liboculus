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

#include <g3log/g3log.hpp>  // needed for CHECK macro

#include "Oculus/Oculus.h"
#include "liboculus/DataTypes.h"

namespace liboculus {

// \todo  A long-term TODO: ImageData, GainData and BearingData are all
// fairly similar in functionality, could reduce the DRY?
template <typename T>
class GainData {
 public:
  typedef T DataType;

  GainData()
    : _data(nullptr), 
      _stride(0),
      _numRanges(0),
      _imageSize(0)
    {;}

  GainData(const GainData &other)  = default;

  GainData(const T *data, uint32_t imageSz, size_t strideBytes, size_t nRanges)
      : _data(data),
        _stride(strideBytes/sizeof(T)),
        _numRanges(nRanges),
        _imageSize(imageSz) {}

  int size() const { return _numRanges; }

  T at(unsigned int i) const {
    CHECK(i < _numRanges) << "Requested gain " << i << " out of range";

    const size_t index = i*_stride;
    CHECK(index*sizeof(T) < _imageSize);

    return _data[index];
  }

  T operator[](unsigned int i) const { return at(i); }

 private:
  const T *_data;
  size_t _stride, _numRanges;
  uint32_t _imageSize;
};

}  // namespace liboculus
