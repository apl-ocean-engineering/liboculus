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

    void receiveStatus( const SonarStatus & status );

    // Simple passthrough
    void setDataRxCallback( DataRx::SimplePingCallback callback )
      { _dataRx.setCallback( callback ); }

    void join();
    void stop();

  protected:

    // Runs in thread
    //void run();

  private:

    std::string _ipAddr;
    std::thread _thread;

    IoServiceThread _ioSrv;
    StatusRx _statusRx;
    DataRx _dataRx;

  };
}
