#include <gtest/gtest.h>

#include "liboculus/SimpleFireMessage.h"

using namespace liboculus;

void validateOculusHeader( const OculusMessageHeader &head )
{
  ASSERT_EQ( head.oculusId, 0x4f53 );
}


TEST(SimpleFireMessage, serialize) {
  SimpleFireMessage msg;

  boost::asio::streambuf b;

  msg.serializeTo(b);

  ASSERT_EQ( b.size(), sizeof(OculusSimpleFireMessage) );

  // De-serialize and validate the message
  {
    OculusSimpleFireMessage msg;

    b.sgetn( (char *)&msg, sizeof(OculusSimpleFireMessage) );
    validateOculusHeader( msg.head );
  }
}
