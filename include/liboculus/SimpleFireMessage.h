#pragma once

#include <boost/asio.hpp>

#include "Oculus/Oculus.h"

namespace liboculus {


// OO wrapper around OculusSimpleFireMessage
class SimpleFireMessage {
public:
  SimpleFireMessage();

  void serialize( boost::asio::streambuf &buffer );

  void setGamma(uint8_t input);

  void setPingRate(uint8_t input);

  void setGainPercent(uint8_t input);

  void setRange(uint8_t input);

private:

  OculusSimpleFireMessage _sfm;

};

}
