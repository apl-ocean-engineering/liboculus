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

// Overlay datatype that enables accessing the image data by
// bearing/range coordinates without copying into another type.

#pragma once

#include <iostream>

#include <g3log/g3log.hpp>  // needed for CHECK macro

#include "DataTypes.h"
#include "Oculus/Oculus.h"

namespace liboculus {

class ImageData {
 public:
  ImageData() 
    : _data(nullptr),
      _imageSize(0),
      _numRanges(0),
      _numBeams(0),
      _dataSize(0) {}

  ImageData(const ImageData &other) = default;

  ImageData( const uint8_t *data,
            uint32_t imageSize,
            uint16_t nRanges,
            uint16_t nBeams,
            uint8_t dataSize )
    : _data(data), 
      _imageSize( imageSize ),
      _numRanges( nRanges ),
      _numBeams( nBeams ),
      _dataSize( dataSize ) {}

  // TODO.  Deal with non-8-bit data somehow
  uint8_t at(unsigned int bearing, unsigned int range) const {
    CHECK(_dataSize == 1) << "Sorry, can only handle 8-bit data right now";
    if (_data == nullptr) {
      return 0;
    }

    // TODO range check
    const unsigned int index = range * _numBeams + bearing;
    CHECK(index < (unsigned int)(_numRanges * _numBeams));

    return ((uint8_t *)_data)[range * _numBeams + bearing];
  }

 private:
  const uint8_t *_data;

  uint16_t _numRanges, _numBeams;
  uint8_t _dataSize;
  uint32_t _imageSize;
};  // class ImageData

}  // namespace liboculus
