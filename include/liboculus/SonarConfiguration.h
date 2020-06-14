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

#pragma once

#include <boost/asio.hpp>

#include "Oculus/Oculus.h"

namespace liboculus {


// OO wrapper around OculusSonarConfiguration
class SonarConfiguration {
public:

  typedef std::function< void( const SonarConfiguration & ) > SonarConfigurationChangedCallback;

  SonarConfiguration();

  void serializeTo( boost::asio::streambuf &buffer ) const;

  void setCallback( SonarConfigurationChangedCallback callback )
    { _callback = callback; }

  void postponeCallback();
  void sendCallback();

  // Setter functions
  void setGamma(double input);

  void setPingRate(PingRateType newRate);

  void setGainPercent(double input);

  void setRange(double input);

  void setWaterTemperature( double degC );

  typedef enum {
    OCULUS_LOW_FREQ = 1,
    OCULUS_HIGH_FREQ = 2
  } OculusFreqMode;

  void setFreqMode(OculusFreqMode input);

private:


  bool _postponeCallback;

  OculusSimpleFireMessage _sfm;

  SonarConfigurationChangedCallback _callback;

};

}
