

#include "liboculus/SonarStatus.h"

#include "libg3logger/g3logger.h"


namespace liboculus {

  using std::string;

  using boost::asio::ip::address_v4;

  SonarStatus::SonarStatus()
  : _statusMutex(),
    _statusUpdateCond(),
    _valid(false)
  {}


  OculusStatusMsg SonarStatus::operator()( void ) const
  {
    std::lock_guard<std::mutex> lock( _statusMutex );

    // Copy constructor?
    // OculusStatusMsg osm;
    // memcpy( (void *)&osm, (void *)&_osm, sizeof(OculusStatusMsg) );

    return _osm;
  }

  boost::asio::ip::address SonarStatus::ipAddr() const {
    std::lock_guard<std::mutex> lock( _statusMutex );
    return address_v4( ntohl( _osm.ipAddr ));
  }

  void SonarStatus::update( const OculusStatusMsg &msg,  sys_time_point msgTime )
  {
    std::lock_guard<std::mutex> lock( _statusMutex );

    memcpy( (void *)&_osm, (void *)&msg, sizeof(OculusStatusMsg) );
    _msgTime = msgTime;

    _statusUpdateCond.notify_all();
  }

  void SonarStatus::dump() const
  {
    std::lock_guard<std::mutex> lock( _statusMutex );

    LOG(DEBUG) << "Device id " << _osm.deviceId << " ; type: " <<  (uint16_t)_osm.deviceType << " ; part num: " << (uint16_t)_osm.partNumber;

    //LOG(DEBUG) << "        Received at: " << _msgTime;
    LOG(DEBUG) << "             Status: " << std::hex << _osm.status;
    LOG(DEBUG) << "      Sonar ip addr: " << boost::asio::ip::address_v4( ntohl(_osm.ipAddr) );
    LOG(DEBUG) << " Sonar connected to: " << boost::asio::ip::address_v4( ntohl(_osm.connectedIpAddr) );

    LOG(DEBUG) << "Versions:";
    LOG(DEBUG) << "   firmwareVersion0: " << std::hex << _osm.versionInfo.firmwareVersion0;
    LOG(DEBUG) << "      firmwareDate0: " << std::hex << _osm.versionInfo.firmwareDate0;

    LOG(DEBUG) << "   firmwareVersion1: " << std::hex << _osm.versionInfo.firmwareVersion1;
    LOG(DEBUG) << "      firmwareDate1: " << std::hex << _osm.versionInfo.firmwareDate1;

    LOG(DEBUG) << "   firmwareVersion2: " << std::hex << _osm.versionInfo.firmwareVersion2;
    LOG(DEBUG) << "      firmwareDate2: " << std::hex << _osm.versionInfo.firmwareDate2;
  }

}
