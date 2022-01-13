
#include <gtest/gtest.h>

#include "test_data.h"

const size_t FileSize = 182016;

TEST(TestData, LoadOneRawPing) {
  // This is known apriori from the file "one_ping.raw"

  auto ping( Oculus_TestData::Load(ONE_RAW_PING));
  ASSERT_EQ(FileSize, ping->size());
}
