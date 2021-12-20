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

#include "liboculus/OculusStructs.h"

namespace liboculus {

SimplePingResult::SimplePingResult(const ByteVector &buffer)
  : MessageHeader(buffer),
    _bearings(),
    _image() 
{
  assert(buffer.size() >= sizeof(OculusSimplePingResult));

  // Bearing data is packed into an array of shorts at the end of the
  // OculusSimpleFireMessage
  const short *bearingData = reinterpret_cast<const short*>(buffer.data() + sizeof(OculusSimplePingResult));
  _bearings = BearingData(bearingData, 
                        ping()->nBeams);

  const uint8_t *imageData = reinterpret_cast<const uint8_t*>(buffer.data() + ping()->imageOffset);
  _image = ImageData(imageData,
                        ping()->imageSize,
                        ping()->nRanges,
                        ping()->nBeams,
                        SizeOfDataSize(ping()->dataSize));
}

bool SimplePingResult::valid() const {
  MessageHeader hdr(_buffer);
  if (!hdr.valid()) {
    LOG(WARNING) << "Header not valid";
    return false;
  }

  int num_pixels = ping()->nRanges * ping()->nBeams;
  size_t expected_size = SizeOfDataSize(ping()->dataSize) * num_pixels;

  if (ping()->imageSize != expected_size) {
    LOG(WARNING) << "ImageSize size in header " << ping()->imageSize
                 << " does not match expected data size of "
                 << expected_size;
    return false;
  }

  CHECK(ping()->imageOffset > sizeof(OculusSimplePingResult));
  return true;
}

void SimplePingResult::dump() const {
  LOG(INFO) << "--------------";
  MessageHeader::dump();
  LOG(INFO) << "        Mode: " << FreqModeToString(fireMsg()->masterMode);

  const int pingRate = PingRateToHz(fireMsg()->pingRate);
  if( pingRate >= 0 ) {
    LOG(INFO) << "   Ping rate: " << pingRate;
  } else {
    LOG(INFO) << "   Ping rate: (unknown) " << static_cast<int>(fireMsg()->pingRate);
  }

  LOG(INFO) << "     Ping ID: " << ping()->pingId;
  LOG(INFO) << "      Status: " << ping()->status;
  LOG(INFO) << "   Ping start time: " << ping()->pingStartTime;

  LOG(INFO) << "   Frequency: " << ping()->frequency;
  LOG(INFO) << " Temperature: " << ping()->temperature;
  LOG(INFO) << "    Pressure: " << ping()->pressure;
  LOG(INFO) << "Spd of Sound: " << ping()->speedOfSoundUsed;
  LOG(INFO) << "   Range res: " << ping()->rangeResolution << " m";

  LOG(INFO) << "   Num range: " << ping()->nRanges;
  LOG(INFO) << "   Num beams: " << ping()->nBeams;

  LOG(INFO) << "  Image size: " << ping()->imageSize;
  LOG(INFO) << "Image offset: " << ping()->imageOffset;
  LOG(INFO) << "   Data size: " << DataSizeToString(ping()->dataSize);
  LOG(INFO) << "Message size: " << ping()->messageSize;
  LOG(INFO) << "--------------";
}

} // namespace liboculus
