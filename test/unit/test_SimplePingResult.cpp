#include <gtest/gtest.h>

#include "test_data.h"

#include "liboculus/SimplePingResult.h"

using namespace liboculus;

void ValidateOneRawPing( const SimplePingResult &ping ) {
  ASSERT_TRUE(ping.valid());
}

TEST( SimplePingResult, ConstructorFromBuffer ) {
  SimplePingResult ping(Oculus_TestData::Load(ONE_RAW_PING));

  ValidateOneRawPing( ping );
}
