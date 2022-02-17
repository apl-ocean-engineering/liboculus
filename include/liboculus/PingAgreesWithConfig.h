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

#include "liboculus/SimplePingResult.h"
#include "liboculus/SonarConfiguration.h"

namespace liboculus {

// \TODO develop better API for returning more descriptive results, esp
// when the ping and config don't agree.
template<typename PingT>
bool checkPingAgreesWithConfig( const SimplePingResult<PingT> &ping,
                                const SonarConfiguration &config ) {
    OculusSimpleFireFlags flags(ping.fireMsg()->flags);

    const auto nBeams = ping.ping()->nBeams;
    // (Not used right now)  const auto nRanges = ping.ping()->nRanges;
    const auto dataSize = ping.ping()->dataSize;

    if (config.get512Beams()) {
        if (nBeams != 512) {
            LOG(WARNING) << "Config expects 512 beams, ping has " << nBeams;
        }
    } else {
        if (nBeams != 256) {
            LOG(WARNING) << "Config expects 256 beams, ping has " << nBeams;
        }
    }

    // Check data size
    if (config.getDataSize() != dataSize) {
        LOG(WARNING) << "Config expected "
                    << 8*SizeOfDataSize(config.getDataSize())
                    << " bit data, data is actually "
                    << 8*SizeOfDataSize(dataSize) << " bit";
    }

    return true;
}

}  // namespace liboculus

