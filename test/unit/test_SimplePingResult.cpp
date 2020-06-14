#include <gtest/gtest.h>

#include "test_data.h"

#include "liboculus/SimplePingResult.h"

using namespace liboculus;

void ValidateOneRawPing( const SimplePingResult &ping ) {
  ASSERT_TRUE( ping.valid() );
}

TEST( SimplePingResult, ConstructorFromBuffer ) {
  std::shared_ptr<MessageBuffer> buffer( Oculus_TestData::LoadMessageBuffer( ONE_RAW_PING ) );
  std::shared_ptr<MessageHeader> header( new MessageHeader( buffer ) );

  SimplePingResult ping( header );
}
