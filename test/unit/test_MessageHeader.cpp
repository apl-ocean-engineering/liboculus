#include <gtest/gtest.h>

#include "test_data.h"

#include "liboculus/SimplePingResult.h"

using namespace liboculus;

void ValidateOneRawPingHeader( const MessageHeader &hdr ) {
  ASSERT_TRUE( hdr.valid() );

  // These are known apriori for one_ping.raw
  ASSERT_EQ( messageSimplePingResult, hdr.msgId() );
  ASSERT_EQ( 0x4f53, hdr.oculusId() );
  ASSERT_EQ( 182000, hdr.payloadSize() );

}

TEST( MessageHeader, ConstructorFromBuffer ) {
  std::shared_ptr<MessageBuffer> buffer( Oculus_TestData::LoadMessageBuffer( ONE_RAW_PING ) );

  MessageHeader header( buffer );
  ValidateOneRawPingHeader( header );

}
