
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

int playbackSonarFile( const std::string &filename, ofstream &output, int stopAfter = -1 );

IoServiceThread ioSrv;

// Catch signals
void signalHandler( int signo ) {
  ioSrv.stop();
}

int main( int argc, char **argv ) {

  libg3logger::G3Logger logger("ocClient");

  CLI::App app{"Simple Oculus Sonar app"};

  int verbosity = 0;
  app.add_flag("-v,--verbose", verbosity, "Additional output (use -vv for even more!)");

  string ipAddr("auto");
  app.add_option("--ip", ipAddr, "IP address of sonar or \"auto\" to automatically detect.");

  string outputFilename("");
  app.add_option("-o,--output", outputFilename, "Saves raw sonar data to specified file.");

  string inputFilename("");
  app.add_option("-i,--input", inputFilename, "Reads raw sonar data from specified file.   Plays file contents rather than contacting \"real\" sonar on network.");

  int stopAfter = -1;
  app.add_option("-n,--frames", stopAfter, "Stop after (n) frames.");


  CLI11_PARSE(app, argc, argv);

  if( verbosity == 1 ) {
    logger.stderrHandle->call( &ColorStderrSink::setThreshold, INFO );
  } else if (verbosity > 1 ) {
    logger.stderrHandle->call( &ColorStderrSink::setThreshold, DEBUG );
  }

  ofstream output;

  if( !outputFilename.empty() ) {
    LOG(DEBUG) << "Opening output file " << outputFilename;
    output.open( outputFilename, ios_base::binary | ios_base::out );

    if( !output.is_open() ) {
      LOG(WARNING) << "Unable to open " << outputFilename << " for output.";
      exit(-1);
    }
  }

  if( !inputFilename.empty() ) {
     playbackSonarFile( inputFilename, output, stopAfter );
     return 0;
   }

   signal(SIGHUP, signalHandler );

  int count = 0;

  LOG(DEBUG) << "Starting loop";

  try {

    std::unique_ptr<StatusRx> statusRx( new StatusRx( ioSrv.service() ) );
    std::unique_ptr<DataRx> dataRx( nullptr );

    if( ipAddr != "auto" ) {
      LOG(WARNING) << "Connecting to sonar with IP address " << ipAddr;
      auto addr( boost::asio::ip::address_v4::from_string( ipAddr ) );

      LOG_IF(FATAL,addr.is_unspecified()) << "Hm, couldn't parse IP address";

      dataRx.reset( new DataRxQueued( ioSrv.service(), addr ) );

      // Setup callback
      dataRx->setCallback( [&]( const shared_ptr<SimplePingResult> &ping ) {
          // Do something
        auto valid = ping->valid();
        LOG(INFO) << "Got " << (valid ? "valid" : "invalid") << " ping";

        if( output.is_open() ) {
          auto const buffer( ping->buffer() );
          output.write( (const char *)buffer->ptr(), buffer->size() );
        }

        count++;
        if( (stopAfter>0) && (count >= stopAfter)) ioSrv.stop();
      });
    }


    statusRx->setCallback( [&]( const std::shared_ptr<SonarStatus> &status ) {
      if( dataRx ) return;

      LOG(DEBUG) << "   ... got status message";
      if( status->valid() ) {
        auto addr( status->ipAddr() );

        LOG(INFO) << "Using detected sonar at IP address " << addr;

        if( verbosity > 0 ) status->dump();

        dataRx.reset( new DataRx( ioSrv.service(), addr ) );

        // Whoops.  How's the DRY?
        dataRx->setCallback( [&]( const shared_ptr<SimplePingResult> &ping ) {
            // Do something
          auto valid = ping->valid();
          LOG(INFO) << "Got " << (valid ? "valid" : "invalid") << " ping";

          if( output.is_open() ) {
            auto const buffer( ping->buffer() );
            output.write( (const char *)buffer->ptr(), buffer->size() );
          }

          count++;
          if( (stopAfter>0) && (count >= stopAfter)) ioSrv.stop();

        });
      } else {
        LOG(DEBUG) << "   ... but it wasn't valid";
      }

    });

    ioSrv.fork();

    // just wait
    ioSrv.join();
  }
  catch (std::exception& e)
  {
    LOG(WARNING) << "Exception: " << e.what();
  }

  if( output.is_open() ) output.close();

  return 0;
}


int playbackSonarFile( const std::string &filename, ofstream &output, int stopAfter ) {
  std::shared_ptr<SonarPlayerBase> player( SonarPlayerBase::OpenFile(filename) );

  if( !player ) {
    LOG(WARNING) << "Unable to open sonar file";
    return -1;
  }

  if( !player->open(filename) ) {
    LOG(INFO) << "Failed to open " << filename;
    return -1;
  }

  int count = 0;
  std::shared_ptr<SimplePingResult> ping( player->nextPing() );
  while( ping && !player->eof() ) {

    ping->valid();

    if( output.is_open() ) {
      auto const buffer( ping->buffer() );
      output.write( (const char *)buffer->ptr(), buffer->size() );
    }

    count++;
    if( (stopAfter > 0) && (count >= stopAfter) ) break;

    ping = player->nextPing();
  }

  LOG(INFO) << count << " sonar packets decoded";

  return 0;
}
