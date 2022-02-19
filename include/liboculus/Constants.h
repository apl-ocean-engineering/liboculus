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
    const uint16_t StatusBroadcastPort = 52102;
    const uint16_t DataPort = 52100;

    const uint8_t PacketHeaderLSB = (OCULUS_CHECK_ID & 0x00FF);
    const uint8_t PacketHeaderMSB = (OCULUS_CHECK_ID & 0xFF00) >> 8;

    namespace Oculus_1200MHz {
        const float ElevationBeamwidthDeg = 20.0;
        const float ElevationBeamwidthRad = 20.0*M_PI/180.0;

        const float AzimuthBeamwidthDeg = 0.6;
        const float AzimuthBeamwidthRad = 0.6*M_PI/180.0;

        const float MaxRange = 40;
    };

    namespace Oculus_2100MHz {
         const float ElevationBeamwidthDeg = 12.0;
         const float ElevationBeamwidthRad = 12.0*M_PI/180.0;

         const float AzimuthBeamwidthDeg = 0.4;
         const float AzimuthBeamwidthRad = 0.4*M_PI/180.0;

         // \todo These shouldn't be fixed, should read from Oculus.h
         // But I don't feel like dealing with their data structure
         const float MaxRange = 10;  // meters
    };

struct FlagBits {
  // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
  // bit 1: 0 = 8 bit data, 1 = 16 bit data
  // bit 2: 0 = wont send gain, 1 = send gain
  // bit 3: 0 = send full return message, 1 = send simple return message
  // bit 4: "gain assistance"?
  // bit 6: use 512 beams (vs 256): email from Blueprint said to set flags |= 0x40

  static const uint8_t RangeAsMeters = (0x01) << 0;
  static const uint8_t Data16Bits    = (0x01) << 1;
  static const uint8_t DoSendGain    = (0x01) << 2;
  static const uint8_t SimpleReturn  = (0x01) << 3;
  static const uint8_t GainAssistance = (0x01) << 4;
  static const uint8_t Do512Beams    = (0x01) << 6;
};

  // There doesn't appear to be an enum for the masterMode (like there is
  // for pingRate and dataSize), so creating our own to match comments in
  // liboculus/thirdparty/Oculus/Oculus.h.
  typedef enum {
    OCULUS_LOW_FREQ = 1,
    OCULUS_HIGH_FREQ = 2
  } OculusFreqMode;


}  // namespace liboculus
