
#include <memory>
#include <thread>
#include <string>

using std::string;

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <libg3logger/g3logger.h>
#include <CLI/CLI.hpp>


#include "liboculus/StatusRx.h"
#include "liboculus/SonarClient.h"


using namespace liboculus;

class IoServiceThread {
public:
    IoServiceThread()
      : _service(),
        _thread() {}

    ~IoServiceThread() {}

    void start()
    {
        if (_thread) return; // running

        _thread.reset(new std::thread(
            boost::bind(&boost::asio::io_service::run, &_service)
        ));
    }

    void stop()
    {
        if (!_thread) return; // stopped

        _service.stop();
        _thread->join();
        _service.reset();
        _thread.reset();
    }

    boost::asio::io_service &service()
    { return _service; }

private:
    boost::asio::io_service _service;
    std::unique_ptr<std::thread> _thread;
};


int main( int argc, char **argv ) {

  libg3log::G3Logger logger("ocClient");

  CLI::App app{"Simple Oculus Sonar app"};

  int verbosity = 0;
  app.add_flag("-v", verbosity, "Additional output (use -vv for even more!)");

  string ipAddr("auto");
  app.add_option("-i,--ip", ipAddr, "IP address of sonar or \"auto\" to automatically detect.");

  CLI11_PARSE(app, argc, argv);

  switch(verbosity) {
          case 1:
                  logger.stderrHandle->call( &ColorStderrSink::setThreshold, INFO );
                  break;
          case 2:
                  logger.stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );
                  break;
  }


  try {
    IoServiceThread ioSrv;

    OsStatusRx statusRx( ioSrv.service() );
    std::unique_ptr<SonarClient> sonarClient( nullptr );

    if( ipAddr != "auto" ) {
      LOG(INFO) << "Connecting to sonar with IP address " << ipAddr;
      auto addr( boost::asio::ip::address_v4::from_string( ipAddr ) );

      LOG_IF(FATAL,addr.is_unspecified()) << "Hm, couldn't parse IP address";

      sonarClient.reset( new SonarClient( ioSrv.service(), addr ) );
    }

    ioSrv.start();

    while(true) {

      if( !sonarClient && ipAddr == "auto") {
        if( statusRx.status().valid() ) {
          auto addr( statusRx.status().ipAddr() );

          LOG(INFO) << "Using detected sonar at IP address " << addr;

          // Is this necessary?
          ioSrv.stop();

          sonarClient.reset( new SonarClient( ioSrv.service(), addr ) );

          ioSrv.start();
        }
      }

      if( sonarClient ) {
        // Do some stuff
        sleep(1);
      }

    }

    ioSrv.stop();

  }
  catch (std::exception& e)
  {
    LOG(WARNING) << "Exception: " << e.what();
  }



}
