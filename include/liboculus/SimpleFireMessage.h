#pragma once

#include <boost/asio.hpp>

#include "Oculus/Oculus.h"

namespace liboculus {


// OO wrapper around OculusSimpleFireMessage
class SimpleFireMessage {
public:
  SimpleFireMessage();

  void serialize( boost::asio::streambuf &buffer );

private:

  OculusSimpleFireMessage _sfm;

};

}
