

#include "liboculus/SonarClient.h"

namespace liboculus {


  SonarClient::SonarClient( const std::string &ipAddr )
      : _ipAddr( ipAddr ),
      _thread(),
      _ioSrv(),
      _statusRx( _ioSrv.service() ),
      _dataRx( _ioSrv.service() )
  {

    _statusRx.setCallback( std::bind( &SonarClient::receiveStatus, this, std::placeholders::_1 ));
    //_dataRx.setCallback( std::bind( &SonarClient::receivePing, this, std::placeholders::_1 ) );

    if( ! _ipAddr.empty() && _ipAddr != "auto" ) {
      LOG(INFO) << "Connecting to sonar with IP address " << _ipAddr;
      auto addr( boost::asio::ip::address_v4::from_string( _ipAddr ) );

      LOG_IF(FATAL,addr.is_unspecified()) << "Couldn't parse IP address" << _ipAddr;

      _dataRx.connect( addr );
    }

  }


  SonarClient::~SonarClient()
  {
    stop();
  }

  void SonarClient::start() {
    _thread = std::thread( [=] { run(); } );
  }

  void SonarClient::join() {
    if( _thread.joinable() ) {
      _thread.join();
    }
  }

  void SonarClient::stop() {
    _ioSrv.stop();
    join();
  }


  // Runs in thread
  void SonarClient::run() {
    LOG(DEBUG) << "Starting SonarClient in thread";
    try {
      _ioSrv.fork();

      _ioSrv.join();
    }
    catch (std::exception& e)
    {
      LOG(WARNING) << "Exception: " << e.what();
    }

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
