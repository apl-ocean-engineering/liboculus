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


  uint8_t at_uint8(unsigned int bearing, unsigned int range) const {
    CHECK(_dataSize == 1) << "This function can only handle 8-bit data, use at_uint16()";
    if ((_data == nullptr) || (bearing >= _numBeams) || (range >= _numRanges)) return 0;

    const size_t index = range * _numBeams + bearing;
    CHECK(index < (unsigned int)(_numRanges * _numBeams));

    return ((uint8_t *)_data)[range * _numBeams + bearing];
  }

    // This function works for either 1 or 2byte data.  
    // For 1-byte data, it's stored in the lower bytes, and the
    // upper byte is always 0
    uint16_t at_uint16(unsigned int bearing, unsigned int range) const {
        if ((_data == nullptr) || (bearing >= _numBeams) || (range >= _numRanges)) return 0;

        const size_t index = range * _numBeams + bearing;
        if(_dataSize == 1) {
            return at_uint8(bearing,range);
        } else if (_dataSize == 2) {
            const size_t offset = index * _dataSize;
            return (_data[offset] | _data[offset+1] << 8);
        }

    return 0;
    }

 private:
  const uint8_t *_data;

  uint16_t _numRanges, _numBeams;
  uint8_t _dataSize;
  uint32_t _imageSize;
};  // class ImageData

}  // namespace liboculus
