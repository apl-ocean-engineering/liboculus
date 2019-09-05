

#include "liboculus/SonarClient.h"

namespace liboculus {


  SonarClient::SonarClient( const std::string &ipAddr )
      : _ioSrv(),
      _statusRx( _ioSrv.service() ),
      _dataRx( _ioSrv.service() )
  {

    _statusRx.setCallback( std::bind( &SonarClient::receiveStatus, this, std::placeholders::_1 ));
    //_dataRx.setCallback( std::bind( &SonarClient::receivePing, this, std::placeholders::_1 ) );

    if( ! ipAddr.empty() && ipAddr != "auto" ) {
      LOG(INFO) << "Connecting to sonar with IP address " << ipAddr;
      auto addr( boost::asio::ip::address_v4::from_string( ipAddr ) );

      LOG_IF(FATAL,addr.is_unspecified()) << "Couldn't parse IP address" << ipAddr;

      _dataRx.connect( addr );
    }

  }


  SonarClient::~SonarClient()
  {
    stop();
    join();
  }

  void SonarClient::start() {
    _ioSrv.fork();
  }

  void SonarClient::join() {
    _ioSrv.join();
  }

  void SonarClient::stop() {
    _ioSrv.stop();
  }

  void SonarClient::receiveStatus( const SonarStatus &status ) {
    if( _dataRx.connected() ) return;

    /// Todo.  Change this to a callback...
    // Attempt auto detection
    if( status.valid() ) {
      auto addr( status.ipAddr() );
      LOG(INFO) << "Using sonar detected at " << addr;
      _dataRx.connect( addr );
    }
  }

}
