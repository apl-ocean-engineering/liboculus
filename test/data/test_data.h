#pragma once

#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "liboculus/SimplePingResult.h"

// NOTE: must be set via CMake
#ifndef TEST_DATA_PATH
#error "TEST_DATA_PATH must be defined for unit test data to be found"
#endif

#define ONE_RAW_PING (TEST_DATA_PATH "/one_ping_8bit.raw")
#define THREE_RAW_PINGS (TEST_DATA_PATH "/three_pings_8bit.raw")

namespace Oculus_TestData {

using liboculus::ByteVector;
using std::vector;

inline std::shared_ptr<ByteVector> Load(const std::string &filename) {
  std::ifstream inf(filename);

  if (!inf.is_open()) return std::make_shared<ByteVector>();

  // This feels a little ... wrong
  inf.seekg(0, std::ios::end);

  const size_t sz = inf.tellg();

  std::string out(sz, '\0');
  inf.seekg(0, std::ios::beg);
  inf.read(&out[0], sz);

  return std::make_shared<ByteVector>(out.begin(), out.end());
}

}  // namespace Oculus_TestData
