#include <gtest/gtest.h>

#include "test_data.h"

#include "liboculus/SimplePingResult.h"

using namespace liboculus;

void ValidateOneRawPing( const SimplePingResult &ping ) {
  ASSERT_TRUE( ping.valid() );
}

TEST( SimplePingResult, ConstructorFromBuffer ) {
  MessageHeader header( Oculus_TestData::LoadMessageBuffer( ONE_RAW_PING ) );
  SimplePingResult ping( header );
  
  ValidateOneRawPing( ping );
}
