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
      _dataSize(0),
      _stride(0),
      _offset(0) {}

  ImageData(const ImageData &other) = default;

  ImageData( const uint8_t *data,
            uint32_t imageSize,
            uint16_t nRanges,
            uint16_t nBeams,
            uint8_t dataSize,
            uint16_t stride = 0,
            uint16_t offset = 0 )
    : _data(data), 
      _imageSize( imageSize ),
      _numRanges( nRanges ),
      _numBeams( nBeams ),
      _dataSize( dataSize ),
      _stride( stride == 0 ? nBeams*dataSize : stride ),  // Stride is in _bytes_
      _offset( offset ) {}


  uint8_t at_uint8(unsigned int beam, unsigned int rangeBin) const {
    CHECK(_dataSize == 1) << "This function can only handle 8-bit data, use at_uint16()";
    if ((_data == nullptr) || (beam >= _numBeams) || (rangeBin >= _numRanges)) return 0;

    // Simplified calculation assumes 1-byte data
    const size_t index = rangeBin * _stride + beam + _offset;
    CHECK(index < _imageSize);
    return ((uint8_t *)_data)[index];
  }

    // This function works for either 1- or 2-byte sonar data
    // For 1-byte, the 8-bit value is simply cast into the 16-bit return
    uint16_t at_uint16(unsigned int beam, unsigned int rangeBin) const {
    if ((_data == nullptr) || (beam >= _numBeams) || (rangeBin >= _numRanges)) return 0;

    if(_dataSize == 1) {
        return at_uint8(beam,rangeBin);
    } else if (_dataSize == 2) {
        const size_t offset = (rangeBin * _stride) + (beam * _dataSize) + _offset;
        CHECK(offset < (_imageSize-1));
        return (_data[offset] | _data[offset+1] << 8);
    }

    return 0;
    }

 private:
  const uint8_t *_data;
  uint32_t _imageSize;
  size_t _numRanges, _numBeams, _stride, _offset;
  uint8_t _dataSize;
};  // class ImageData

}  // namespace liboculus
