#pragma once

#include "Oculus/Oculus.h"

namespace liboculus {

  inline const char *DataSizeToString( DataSizeType d ) {
    switch(d) {
      case dataSize8Bit:
        return "8-bit";
      case dataSize16Bit:
        return "16-bit";
      case dataSize24Bit:
        return "24-bit";
      case dataSize32Bit:
        return "32-bit";
    }

    return "unknown";
  }

  inline size_t SizeOfDataSize( DataSizeType d ) {
    switch(d) {
      case dataSize8Bit:
        return 1;
      case dataSize16Bit:
        return 2;
      case dataSize24Bit:
        return 3;
      case dataSize32Bit:
        return 4;
    }

    return 0;
  }


}
