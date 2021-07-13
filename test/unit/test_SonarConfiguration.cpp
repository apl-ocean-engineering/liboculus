#include <gtest/gtest.h>

#include "liboculus/SonarConfiguration.h"

using namespace liboculus;

void validateOculusHeader( const OculusMessageHeader &head )
{
  ASSERT_EQ( head.oculusId, OCULUS_CHECK_ID);  // 0x4f53

  // \TODO  exapand validity testing
}


TEST(SonarConfiguration, serialize) {
  SonarConfiguration msg;

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
