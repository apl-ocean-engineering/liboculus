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

#include <cmath>
#include <cstddef>  // for size_t
#include <string>
#include <vector>

#include "Oculus/Oculus.h"

namespace liboculus {
const uint16_t StatusBroadcastPort = 52102;
const uint16_t DataPort = 52100;

const uint8_t PacketHeaderLSB = (OCULUS_CHECK_ID & 0x00FF);
const uint8_t PacketHeaderMSB = (OCULUS_CHECK_ID & 0xFF00) >> 8;

#define DEG2RAD(x) (x * M_PI / 180.0)

//===================================================================
//
// New constants API which separates constants by both model
// and frequency band

namespace Oculus_M750d {
namespace Freq_750kHz {
const float ElevationBeamwidthDeg = 20.0;
const float ElevationBeamwidthRad = DEG2RAD(ElevationBeamwidthDeg);

const float AzimuthBeamwidthDeg = 1.0;
const float AzimuthBeamwidthRad = DEG2RAD(AzimuthBeamwidthDeg);

const float MaxRange = 120;
};  // namespace Freq_750kHz

namespace Freq_1200kHz {
const float ElevationBeamwidthDeg = 12.0;
const float ElevationBeamwidthRad = DEG2RAD(ElevationBeamwidthDeg);

const float AzimuthBeamwidthDeg = 0.6;
const float AzimuthBeamwidthRad = DEG2RAD(AzimuthBeamwidthDeg);

//! NOTE:(by LinZhao) for old generation 1.2MHz,
//        M750d max range is 40m, M1200d max range is 30m
//        Maybe set a specfic name for 1.2MHz, since
//        the beamwidth changes between generation and model ?
const float MaxRange = 40;
};  // namespace Freq_1200kHz
};  // namespace Oculus_M750d

namespace Oculus_M1200d {
namespace Freq_1200kHz {
const float ElevationBeamwidthDeg = 20.0;
const float ElevationBeamwidthRad = DEG2RAD(ElevationBeamwidthDeg);

const float AzimuthBeamwidthDeg = 0.6;
const float AzimuthBeamwidthRad = DEG2RAD(AzimuthBeamwidthDeg);

const float MaxRange = 40;
};  // namespace Freq_1200kHz

namespace Freq_2100kHz {
const float ElevationBeamwidthDeg = 12.0;
const float ElevationBeamwidthRad = DEG2RAD(ElevationBeamwidthDeg);

const float AzimuthBeamwidthDeg = 0.4;
const float AzimuthBeamwidthRad = DEG2RAD(AzimuthBeamwidthDeg);

const float MaxRange = 10;
};  // namespace Freq_2100kHz
};  // namespace Oculus_M1200d

namespace Oculus_M3000d {
namespace Freq_1200kHz {
const float ElevationBeamwidthDeg = 20.0;
const float ElevationBeamwidthRad = DEG2RAD(ElevationBeamwidthDeg);

const float AzimuthBeamwidthDeg = 0.6;
const float AzimuthBeamwidthRad = DEG2RAD(AzimuthBeamwidthDeg);

const float MaxRange = 30;
};  // namespace Freq_1200kHz

namespace Freq_3000kHz {
const float ElevationBeamwidthDeg = 20.0;
const float ElevationBeamwidthRad = DEG2RAD(ElevationBeamwidthDeg);

const float AzimuthBeamwidthDeg = 0.4;
const float AzimuthBeamwidthRad = DEG2RAD(AzimuthBeamwidthDeg);

const float MaxRange = 5;
};  // namespace Freq_3000kHz
};  // namespace Oculus_M3000d

//===================================================================
//
// "Old"" constants API which only considered nominal frequency.
// So it can't handle the case where different models may have
// different performance at the same nominal center frequency.

// For backwards compatibility
namespace Oculus_750KHz = Oculus_M750d::Freq_750kHz;
namespace Oculus_1200MHz = Oculus_M1200d::Freq_1200kHz;
namespace Oculus_2100MHz = Oculus_M1200d::Freq_2100kHz;
namespace Oculus_3000MHz = Oculus_M3000d::Freq_3000kHz;
struct FlagBits {
  // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
  // bit 1: 0 = 8 bit data, 1 = 16 bit data
  // bit 2: 0 = wont send gain, 1 = send gain
  // bit 3: 0 = send full return message, 1 = send simple return message
  // bit 4: "gain assistance"?
  // bit 6: use 512 beams (vs 256): email from Blueprint said to set flags |=
  // 0x40

  static const uint8_t RangeAsMeters = (0x01) << 0;
  static const uint8_t Data16Bits = (0x01) << 1;
  static const uint8_t DoSendGain = (0x01) << 2;
  static const uint8_t SimpleReturn = (0x01) << 3;
  static const uint8_t GainAssistance = (0x01) << 4;
  static const uint8_t Do512Beams = (0x01) << 6;
};

// There doesn't appear to be an enum for the masterMode (like there is
// for pingRate and dataSize), so creating our own to match comments in
// liboculus/thirdparty/Oculus/Oculus.h.
typedef enum { OCULUS_LOW_FREQ = 1, OCULUS_HIGH_FREQ = 2 } OculusFreqMode;

}  // namespace liboculus
