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

// Utility conversions for data types defined in Oculus.h

#pragma once

#include <cstddef>  // for size_t
#include <string>
#include <vector>
#include <cmath>

#include "Oculus/Oculus.h"

namespace liboculus {

typedef std::vector<uint8_t> ByteVector;

template <typename T>
T deg2rad(const T &value) { return M_PI/180.0 * value; }

inline const char *DataSizeToString( DataSizeType d ) {
  switch(d) {
    case dataSize8Bit:
      return "8-bit";
    case dataSize16Bit:
      return "16-bit";
    case dataSize24Bit:
      return "24-bit";
    case dataSize32Bit:
      return "32-bit";
  }

  return "unknown";
}

inline size_t SizeOfDataSize( DataSizeType d ) {
  switch(d) {
    case dataSize8Bit:
      return 1;
    case dataSize16Bit:
      return 2;
    case dataSize24Bit:
      return 3;
    case dataSize32Bit:
      return 4;
  }

  return 0;
}

//=== Message types

inline const char *MessageTypeToString( OculusMessageType t ) {
  switch(t) {
    case messageSimpleFire:
        return "messageSimpleFire";
    case messagePingResult:
        return "messagePingResult";
    case messageSimplePingResult:
        return "messageSimplePingResult";
    case messageUserConfig:
        return "messageUserConfig";
    case messageLogs:
        return "messageLogs";
    case messageDummy:
        return "messageDummy";
  }

    return "(unknown)";
}

//=== Ping rate to string

inline unsigned int PingRateToHz( PingRateType p ) {
  switch(p) {
    case pingRateNormal:   return 10;
    case pingRateHigh:     return 15;
    case pingRateHighest:  return 40;
    case pingRateLow:      return 5;
    case pingRateLowest:   return 2;
    case pingRateStandby:  return 0;
  }

  return -1;
}

inline unsigned int PingRateToHz( int p ) {
  return PingRateToHz( static_cast<PingRateType>(p) );
}

inline std::string FreqModeToString( uint8_t mode  ) {
  if( mode == 1 ) {
    return "Low Freq";
  } else if (mode == 2 ) {
    return "High Freq";
  }

  return "(unknown)";
}

}
