

#include <boost/asio.hpp>

#include "liboculus/SimpleFireMessage.h"


namespace liboculus {

  SimpleFireMessage::SimpleFireMessage()
  {
    memset( &_sfm, 0, sizeof(OculusSimpleFireMessage));

    _sfm.head.msgId       = messageSimpleFire;
    _sfm.head.srcDeviceId = 0;
    _sfm.head.dstDeviceId = 0;
    _sfm.head.oculusId    = 0x4f53;

    // OculusMessageHeader head;     // The standard message header
    //
    // uint8_t masterMode;           // mode 0 is flexi mode, needs full fire message (not available for third party developers)
    //                               // mode 1 - Low Frequency Mode (wide aperture, navigation)
    //                               // mode 2 - High Frequency Mode (narrow aperture, target identification)
    // PingRateType pingRate;        // Sets the maximum ping rate.
    // uint8_t networkSpeed;         // Used to reduce the network comms speed (useful for high latency shared links)
    // uint8_t gammaCorrection;      // 0 and 0xff = gamma correction = 1.0
    //                               // Set to 127 for gamma correction = 0.5
    // uint8_t flags;                // bit 0: 0 = interpret range as percent, 1 = interpret range as meters
    //                               // bit 1: 0 = 8 bit data, 1 = 16 bit data
    //                               // bit 2: 0 = wont send gain, 1 = send gain
    //                               // bit 3: 0 = send full return message, 1 = send simple return message
    // double range;                 // The range demand in percent or m depending on flags
    // double gainPercent;           // The gain demand
    // double speedOfSound;          // ms-1, if set to zero then internal calc will apply using salinity
    // double salinity;              // ppt, set to zero if we are in fresh water

    // Initial values
    _sfm.gammaCorrection = 0.5; //gamma;
    _sfm.pingRate        = pingRateHigh;
    _sfm.masterMode      = 2;  // HF mode
    _sfm.range           = 10; // 40m
    _sfm.gainPercent     = 50;  //gain;
    _sfm.flags          =  0x09;

    _sfm.speedOfSound    = 1500;  // m/s  0 for automatic calculation speedOfSound;
    _sfm.salinity        = 0;  // ppt; Freshwater salinity;
  }

  void SimpleFireMessage::serialize( boost::asio::streambuf &stream )
  {
    stream.sputn( (char *)&_sfm, sizeof(OculusSimpleFireMessage));
  }


}
