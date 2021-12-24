#include <gtest/gtest.h>

#include "test_data.h"

#include "liboculus/SimplePingResult.h"

using namespace liboculus;

void ValidateOneRawPingHeader(const MessageHeader &hdr) {
  ASSERT_TRUE(hdr.valid());

  // These are known apriori for one_ping.raw
  ASSERT_EQ(messageSimplePingResult, hdr.msgId());
  ASSERT_EQ(OCULUS_CHECK_ID, hdr.oculusId());
  ASSERT_EQ(182000, hdr.payloadSize());
}

TEST(MessageHeader, ConstructorFromBuffer) {
  MessageHeader header(Oculus_TestData::Load(ONE_RAW_PING));
  ValidateOneRawPingHeader(header);
}
