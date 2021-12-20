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

class BearingData {
 public:
  BearingData()
    : _data(nullptr), _numBeams(0)
    {;}

  BearingData(const BearingData &other)  = default;

  BearingData(const int16_t *data, int nBeams)
      : _data(data),
        _numBeams(nBeams) {}

  int size() const { return _numBeams; }

  // Returns bearing in degrees
  //
  // From Oculus.h:
  //   "The bearings to each of the beams in 0.01 degree resolution"
  //
  float at(unsigned int i) const {
    CHECK(i < _numBeams) << "Requested beam " << i << " out of range";
    return _data[i] / 100.0;
  }

  float at_rad(unsigned int i) const {
    return deg2rad(at(i));
  }

 private:
  uint16_t _numBeams;
  const int16_t *_data;
};

}  // namespace liboculus
