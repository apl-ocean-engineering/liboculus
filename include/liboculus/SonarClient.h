#pragma once

#include <string>
#include <thread>
#include "liboculus/IoServiceThread.h"

#include "liboculus/StatusRx.h"
#include "liboculus/DataRx.h"

namespace liboculus {


  class SonarClient {
  public:

    SonarClient( const std::string &ipAddr = "" );

    ~SonarClient();

    void start();
    void join();
    void stop();

    // Simple passthrough
    void setDataRxCallback( DataRx::SimplePingCallback callback )
      { _dataRx.setCallback( callback ); }

  protected:

    void receiveStatus( const SonarStatus & status );

  private:

    std::string _ipAddr;

    IoServiceThread _ioSrv;
    StatusRx _statusRx;
    DataRx _dataRx;

  };
}
