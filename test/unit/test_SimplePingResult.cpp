#include <gtest/gtest.h>

#include "test_data.h"

#include "liboculus/SimplePingResult.h"

using namespace liboculus;

void ValidateOneRawPing( const SimplePingResultV1 &ping ) {
  ASSERT_TRUE(ping.valid());
}

TEST( SimplePingResult, ConstructorFromBuffer ) {
  SimplePingResultV1 ping(Oculus_TestData::Load(ONE_RAW_PING));

  ValidateOneRawPing( ping );
}
