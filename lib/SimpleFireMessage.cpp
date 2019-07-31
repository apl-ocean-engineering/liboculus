

#include <boost/asio.hpp>

#include "liboculus/SimpleFireMessage.h"


namespace liboculus {

  SimpleFireMessage::SimpleFireMessage()
  {
    memset( &_sfm, 0, sizeof(OculusSimpleFireMessage));

    _sfm.head.oculusId    = 0x4f53;
    _sfm.head.msgId       = messageSimpleFire;
    _sfm.head.srcDeviceId = 0;
    _sfm.head.dstDeviceId = 0;
    _sfm.head.payloadSize = sizeof(OculusSimpleFireMessage) - sizeof(OculusMessageHeader);

    // OculusMessageHeader head;     // The standard message header
    //
    // uint8_t masterMode;           // mode 0 is flexi mode, needs full fire message (not available for third party developers)
    //                               // mode 1 - Low Frequency Mode (wide aperture, navigation)
    //                               // mode 2 - High Frequency Mode (narrow aperture, target identification)
    // PingRateType pingRate;        // Sets the maximum ping rate.
    // uint8_t networkSpeed;         // Used to reduce the network comms speed (useful for high latency shared links)
    // uint8_t gammaCorrection;      // 0 and 0xff = gamma correction = 1.0
    //                               // Set to 127 for gamma correction = 0.5

    // double range;                 // The range demand in percent or m depending on flags
    // double gainPercent;           // The gain demand
    // double speedOfSound;          // ms-1, if set to zero then internal calc will apply using salinity
    // double salinity;              // ppt, set to zero if we are in fresh water

    _sfm.masterMode      = 2;

    _sfm.networkSpeed = 0xff;

    // Initial values
    _sfm.gammaCorrection = 127; //gamma;
    _sfm.pingRate        = pingRateLowest;
    _sfm.range           = 2; // Meters
    _sfm.gainPercent     = 50; // gain;

    // uint8_t flags;                // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
    //                               // bit 1: 0 = 8 bit data, 1 = 16 bit data
    //                               // bit 2: 0 = wont send gain, 1 = send gain
    //                               // bit 3: 0 = send full return message, 1 = send simple return message

    _sfm.flags          =  0x19; // Send simple return msg; range in meters

    _sfm.speedOfSound    = 0.0;  // m/s  0 for automatic calculation speedOfSound;
    _sfm.salinity        = 0.0;  // ppt; Freshwater salinity;
  }

  // need to integrate flags into dyanmic reconfig

  void SimpleFireMessage::setRange(uint8_t input)
  {
    // 40 meters is the max range for the 1200d model
    // may need to use a double instead of uint8_t (depends on flags)
    if (input <= 40 && input > 0) {
      _sfm.gammaCorrection = input;
    }
  }

  void SimpleFireMessage::setGainPercent(uint8_t input)
  {
    if (input <= 100 && input > 0) {
      _sfm.gainPercent = input;
    }
  }

  void SimpleFireMessage::setGamma(uint8_t input)
  {
    if (input <= 127 && input > 0) {
      _sfm.gammaCorrection = input;
    }
  }

  void SimpleFireMessage::setPingRate(uint8_t input)
  {
    PingRateType newRate = PingRateType(input);
    _sfm.pingRate = newRate;
  }

  void SimpleFireMessage::serialize( boost::asio::streambuf &stream )
  {
    stream.sputn( (char *)&_sfm, sizeof(OculusSimpleFireMessage));
  }


}
