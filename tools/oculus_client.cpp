
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

// Make these global so signal handler can access it
std::unique_ptr< SonarClient > _client;
bool doStop = false;

// Catch signals
void signalHandler( int signo ) {
  if( _client ) _client->stop();
  doStop = true;
}

int main( int argc, char **argv ) {

  libg3logger::G3Logger logger("ocClient");

  CLI::App app{"Simple Oculus Sonar app"};

  int verbosity = 0;
  app.add_flag("-v,--verbose", verbosity, "Additional output (use -vv for even more!)");

  string ipAddr("auto");
  app.add_option("ip", ipAddr, "IP address of sonar or \"auto\" to automatically detect.");

  string outputFilename("");
  app.add_option("-o,--output", outputFilename, "Saves raw sonar data to specified file.");

  string inputFilename("");
  app.add_option("-i,--input", inputFilename, "Reads raw sonar data from specified file.   Plays file contents rather than contacting \"real\" sonar on network.");

  int stopAfter = -1;
  app.add_option("-n,--frames", stopAfter, "Stop after (n) frames.");


  CLI11_PARSE(app, argc, argv);

  if( verbosity == 1 ) {
    logger.setLevel( INFO );
  } else if (verbosity > 1 ) {
    logger.setLevel( DEBUG );
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

  // If playing back an input file, run a different main loop ...
  if( !inputFilename.empty() ) {
     playbackSonarFile( inputFilename, output, stopAfter );
     return 0;
   }

  int count = 0;

  signal(SIGHUP, signalHandler );

  LOG(DEBUG) << "Starting loop";

  SonarConfiguration config;
  config.setPingRate( pingRateNormal );

  _client.reset( new SonarClient(config, ipAddr) );

  _client->setDataRxCallback( [&]( const SimplePingResult &ping ) {

    auto valid = ping.valid();
    //LOG(INFO) << "Got " << (valid ? "valid" : "invalid") << " ping";

    if( !valid ) {
      LOG(DEBUG) << "Got invalid ping";
      return;
    }

    ping.dump();

    if( output.is_open() ) {
      output.write( (const char *)ping.ptr(), ping.size() );
    }

    count++;
    if( (stopAfter>0) && (count >= stopAfter)) _client->stop();

  });

  _client->start();

  // Imprecise statistic for now...
  int lastCount = 0;
  while( !doStop ) {
    auto c = count;

    LOG(INFO) << "Received pings at " << c-lastCount << " Hz";

    lastCount = c;
    sleep(1);
  }

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
  SimplePingResult ping;
  while( player->nextPing(ping) && !player->eof() ) {

    if( !ping.valid() ) {
      LOG(WARNING) << "Invalid ping";
      continue;
    }

    if( output.is_open() ) {
      output.write( (const char *)ping.ptr(), ping.size() );
    }

    count++;
    if( (stopAfter > 0) && (count >= stopAfter) ) break;
  }

  LOG(INFO) << count << " sonar packets decoded";

  return 0;
}
