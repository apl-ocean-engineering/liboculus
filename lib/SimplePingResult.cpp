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

#include "liboculus/SimplePingResult.h"

namespace liboculus {


  void SimplePingResult::dump() const {
    LOG(DEBUG) << "--------------";
    LOG(DEBUG) << "        Mode: " << oculusFireMsg()->masterMode;
    LOG(DEBUG) << "   Ping rate: " << oculusFireMsg()->pingRate;

    LOG(DEBUG) << "     Ping ID: " << oculusPing()->pingId;
    LOG(DEBUG) << "      Status: " << oculusPing()->status;
    LOG(DEBUG) << "   Ping start time: " << oculusPing()->pingStartTime;

    LOG(DEBUG) << "   Frequency: " << oculusPing()->frequency;
    LOG(DEBUG) << " Temperature: " << oculusPing()->temperature;
    LOG(DEBUG) << "    Pressure: " << oculusPing()->pressure;
    LOG(DEBUG) << "Spd of Sound: " << oculusPing()->speedOfSoundUsed;
    LOG(DEBUG) << "   Range res: " << oculusPing()->rangeResolution << " m";

    LOG(DEBUG) << "   Num range: " << oculusPing()->nRanges;
    LOG(DEBUG) << "   Num beams: " << oculusPing()->nBeams;

    LOG(DEBUG) << "  Image size: " << oculusPing()->imageSize;
    LOG(DEBUG) << "Image offset: " << oculusPing()->imageOffset;
    LOG(DEBUG) << "   Data size: " << DataSizeToString(oculusPing()->dataSize);
    LOG(DEBUG) << "Message size: " << oculusPing()->messageSize;
    LOG(DEBUG) << "--------------";
  }


} // namespace liboculus
