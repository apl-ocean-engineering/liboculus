
#include <memory>
#include <thread>
#include <string>
#include <fstream>

using std::string;

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <libg3logger/g3logger.h>
#include <CLI/CLI.hpp>

#include "liboculus/SonarClient.h"
#include "liboculus/SonarPlayer.h"


using namespace liboculus;

using std::ofstream;
using std::ios_base;

int playbackSonarFile( const std::string &filename, ofstream &output, int stopAfter = -1 );

// Make this global so signal handler can access it
std::unique_ptr< SonarClient > _client;

// Catch signals
void signalHandler( int signo ) {
  if( _client ) _client->stop();
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

   //signal(SIGHUP, signalHandler );

  int count = 0;

  LOG(DEBUG) << "Starting loop";

  _client.reset( new SonarClient(ipAddr) );

  _client->setDataRxCallback( [&]( const shared_ptr<SimplePingResult> &ping ) {
      // Do something
    auto valid = ping->valid();
    LOG(INFO) << "Got " << (valid ? "valid" : "invalid") << " ping";

    if( !valid ) return;

    ping->dump();

    if( output.is_open() ) {
      auto const buffer( ping->buffer() );
      output.write( (const char *)buffer->ptr(), buffer->size() );
    }

    count++;
    if( (stopAfter>0) && (count >= stopAfter)) _client->stop();
  });

  _client->start();
  _client->join();

  if( output.is_open() ) output.close();

  LOG(INFO) << "At exit";

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
