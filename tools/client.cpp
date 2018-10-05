
#include <memory>
#include <thread>
#include <string>
#include <fstream>

using std::string;

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <libg3logger/g3logger.h>
#include <CLI/CLI.hpp>


#include "liboculus/StatusRx.h"
#include "liboculus/DataRx.h"
#include "liboculus/IoServiceThread.h"
#include "liboculus/SonarPlayer.h"


using namespace liboculus;

using std::ofstream;
using std::ios_base;

int playbackSonarFile( const std::string &filename );


int main( int argc, char **argv ) {

  libg3logger::G3Logger logger("ocClient");

  CLI::App app{"Simple Oculus Sonar app"};

  int verbosity = 0;
  app.add_flag("-v", verbosity, "Additional output (use -vv for even more!)");

  string ipAddr("auto");
  app.add_option("--ip", ipAddr, "IP address of sonar or \"auto\" to automatically detect.");

  string outputFilename("");
  app.add_option("-o,--output", outputFilename, "Filename to save sonar data to.");

  string inputFilename("");
  app.add_option("-i,--input", inputFilename, "Filename to read sonar data from.");

  CLI11_PARSE(app, argc, argv);

  if( verbosity == 1 ) {
    logger.stderrHandle->call( &ColorStderrSink::setThreshold, INFO );
  } else if (verbosity > 1 ) {
    logger.stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );
  }


  if( !inputFilename.empty() ) {
    return playbackSonarFile( inputFilename );
  }


  ofstream output;

  if( !outputFilename.empty() ) {
    output.open( outputFilename, ios_base::binary | ios_base::out );

    if( !output.is_open() ) {
      LOG(WARNING) << "Unable to open " << outputFilename << " for output.";
      exit(-1);
    }
  }


  bool notDone = true;

  LOG(DEBUG) << "Starting loop";

  try {
    IoServiceThread ioSrv;

    std::unique_ptr<StatusRx> statusRx( new StatusRx( ioSrv.service() ) );
    std::unique_ptr<DataRx> dataRx( nullptr );

    if( ipAddr != "auto" ) {
      LOG(INFO) << "Connecting to sonar with IP address " << ipAddr;
      auto addr( boost::asio::ip::address_v4::from_string( ipAddr ) );

      LOG_IF(FATAL,addr.is_unspecified()) << "Hm, couldn't parse IP address";

      dataRx.reset( new DataRx( ioSrv.service(), addr ) );
    }

    ioSrv.fork();

    while( notDone ) {

      while( !dataRx ) {

        LOG(DEBUG) << "Need to find the sonar.  Waiting for sonar...";
        if( statusRx->status().wait_for(std::chrono::seconds(1)) ) {

          LOG(DEBUG) << "   ... got status message";
          if( statusRx->status().valid() ) {
            auto addr( statusRx->status().ipAddr() );

            LOG(INFO) << "Using detected sonar at IP address " << addr;

            if( verbosity > 0 ) statusRx->status().dump();

            dataRx.reset( new DataRx( ioSrv.service(), addr ) );

          } else {
            LOG(DEBUG) << "   ... but it wasn't valid";
          }

        } else {
          // Failed to get status, try again.
        }

      }

      shared_ptr<SimplePingResult> ping;

      dataRx->queue().wait_and_pop( ping );

        // Do something
      auto valid = ping->validate();
      LOG(INFO) << "Got " << (valid ? "valid" : "invalid") << " ping";

      if( output.is_open() ) {
        output.write( (const char *)ping->data(), ping->dataSize() );
      }

    }

    ioSrv.stop();

  }
  catch (std::exception& e)
  {
    LOG(WARNING) << "Exception: " << e.what();
  }

  if( output.is_open() ) output.close();


}


int playbackSonarFile( const std::string &filename ) {
  SonarPlayer player;
  if( !player.open(filename) ) {
    LOG(INFO) << "Failed to open " << filename;
    return -1;
  }

  std::shared_ptr<SimplePingResult> ping( player.nextPing() );
  while( ping ) {
    ping->validate();

    ping = player.nextPing();
  }

  return 0;
}
