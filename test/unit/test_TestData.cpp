
#include <gtest/gtest.h>

#include "test_data.h"

const size_t FileSize = 182016;

TEST( TestData, LoadOneRawPing ) {
  // This is known apriori from the file "one_ping.raw"

  std::vector<char> ping( Oculus_TestData::Load( ONE_RAW_PING ) );
  ASSERT_EQ( FileSize, ping.size() );
}

TEST( TestData, LoadOneRawPingMessageBuffer ) {
  std::shared_ptr<liboculus::MessageBuffer> buffer( Oculus_TestData::LoadMessageBuffer( ONE_RAW_PING ) );

  ASSERT_EQ( FileSize, buffer->size() );
}
