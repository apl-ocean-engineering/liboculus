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

bool SimplePingResult::valid() const {
  if (!MessageHeader::valid()) {
    return false;
  }

  int num_pixels = oculusPing()->nRanges * oculusPing()->nBeams;
  size_t expected_size = SizeOfDataSize(oculusPing()->dataSize) * num_pixels;

  if (oculusPing()->imageSize != expected_size) {
    LOG(WARNING) << "ImageSize size in header " << oculusPing()->imageSize
                 << " does not match expected data size of "
                 << expected_size;
    return false;
  }

  // size_t totalSize = expected_size + _msg.imageOffset;
  // if(_msg.messageSize != totalSize) {
  //   LOG(WARNING) << "Message size " << _msg.messageSize << " does not match
  //   expected message size of " << totalSize; return _valid;
  // }

  CHECK(oculusPing()->imageOffset > sizeof(OculusSimplePingResult));
  return true;
}

void SimplePingResult::dump() const {
  LOG(DEBUG) << "--------------";
  LOG(DEBUG) << "        Mode: " << FreqModeToString(oculusFireMsg()->masterMode);

  const int pingRate = PingRateToHz(oculusFireMsg()->pingRate);
  if( pingRate >= 0 ) {
    LOG(DEBUG) << "   Ping rate: " << pingRate;
  } else {
    LOG(DEBUG) << "   Ping rate: (unknown) " << static_cast<int>(oculusFireMsg()->pingRate);
  }

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
