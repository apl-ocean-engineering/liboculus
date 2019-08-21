#pragma once

#include <boost/asio.hpp>

#include "Oculus/Oculus.h"

namespace liboculus {


// OO wrapper around OculusSimpleFireMessage
class SimpleFireMessage {
public:
  SimpleFireMessage();

  void serialize( boost::asio::streambuf &buffer );

  void setGamma(double input);

  void setPingRate(double input);

  void setGainPercent(double input);

  void setRange(double input);

  void setMasterMode(double input);

private:

  OculusSimpleFireMessage _sfm;

};

}
