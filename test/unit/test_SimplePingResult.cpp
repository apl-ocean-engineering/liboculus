#include <gtest/gtest.h>

#include "test_data.h"

#include "liboculus/SimplePingResult.h"

using namespace liboculus;

void ValidateOneRawPing( const SimplePingResult &ping ) {
  ASSERT_TRUE( ping.header().valid() );

}

TEST( SimplePingResult, ConstructorFromBuffer ) {
  std::string pingData = Oculus_TestData::Load( ONE_RAW_PING );

  SimplePingResult ping( pingData.c_str() );
}

TEST( SimplePingResult, ConstructorFromMessageHeader ) {
  std::string pingData = Oculus_TestData::Load( ONE_RAW_PING );

  MessageHeader header( pingData.c_str() );
  SimplePingResult ping( header );

  // Remember it doesn't have other data yet
}
