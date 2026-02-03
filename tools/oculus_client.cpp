#include <chrono>
#include <csignal>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

using std::string;

#include "liboculus/thirdparty/CLI11/CLI11.hpp"
#include <boost/asio.hpp>

#include "spdlog/spdlog.h"

#include "liboculus/DataRx.h"
#include "liboculus/IoServiceThread.h"
#include "liboculus/Logger.h"
#include "liboculus/PingAgreesWithConfig.h"
#include "liboculus/SonarPlayer.h"
#include "liboculus/StatusRx.h"

using std::ios_base;
using std::ofstream;
using std::shared_ptr;

using liboculus::DataRx;
using liboculus::IoServiceThread;
using liboculus::SimplePingResult;
using liboculus::SonarConfiguration;
using liboculus::SonarPlayerBase;
using liboculus::SonarStatus;
using liboculus::StatusRx;
// using liboculus::SonarPlayer;

using std::cout;

int playbackSonarFile(const std::string &filename, ofstream &output,
                      int stopAfter = -1);

// Make these global so signal handler can access it
std::unique_ptr<liboculus::IoServiceThread> _io_thread;
bool doStop = false;

// Catch signals
void signalHandler(int signo) {
  if (_io_thread)
    _io_thread->stop();
  doStop = true;
}

double mean_image_intensity(const liboculus::ImageData &imageData) {
  double f = 0;
  for (int r = 0; r < imageData.nRanges(); ++r) {
    for (int a = 0; a < imageData.nBeams(); ++a) {
      f += imageData.at_uint32(a, r);
    }
  }
  f /= (imageData.nRanges() * imageData.nBeams());
  return f;
}

int main(int argc, char **argv) {

  // Configure both liboculus and occlient to use the same
  // sink to stdout
  auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

  liboculus::Logger::add_sink(stdout_sink);
  spdlog::set_default_logger(
      std::make_shared<spdlog::logger>("occlient", stdout_sink));

  // Or simpler, tell liboculus to use the default logger
  // liboculus::Logger::set_logger( spdlog::default_logger() );

  CLI::App app{"Simple Oculus Sonar app"};

  int verbosity = 0;
  app.add_flag("-v,--verbose", verbosity,
               "Additional output (use -vv for even more!)");

  string ipAddr("auto");
  app.add_option("ip", ipAddr,
                 "IP address of sonar or \"auto\" to automatically detect.");

  string outputFilename("F:\\Projects\\liboculus\\test\\data\\out.raw");
  app.add_option("-o,--output", outputFilename,
                 "Saves raw sonar data to specified file.");

  // Playback currently not working
  string inputFilename("F:\\Projects\\liboculus\\test\\data\\one_ping_8bit.raw");
  app.add_option("-i,--input", inputFilename,
                 "Reads raw sonar data from specified file.   Plays file "
                 "contents rather than contacting \"real\" sonar on network.");

  int bitDepth(8);
  app.add_option("-b,--bits", bitDepth, "Bit depth oof data (8,16,32)");

  int stopAfter = -1;
  app.add_option("-n,--frames", stopAfter, "Stop after (n) frames.");

  float range = 4;
  app.add_option("-r,--range", range, "Range in meters");

  float gain = 50;
  app.add_option("-g, --gain", gain, "Gain as a percentage (1-100)");

  CLI11_PARSE(app, argc, argv);

  if (verbosity == 1) {
    spdlog::set_level(spdlog::level::debug);
  } else if (verbosity > 1) {
    spdlog::set_level(spdlog::level::trace);
  }

  if ((bitDepth != 8) && (bitDepth != 16) && (bitDepth != 32)) {
    spdlog::error("Invalid bit depth {}", bitDepth);
    exit(-1);
  }

  if ((gain < 1) || (gain > 100)) {
    spdlog::error("Invalid gain {}; should be in the range of 1-100", gain);
  }

  ofstream output;

  if (!outputFilename.empty()) {
    spdlog::debug("Opening output file {}", outputFilename);
    output.open(outputFilename, ios_base::binary | ios_base::out);

    if (!output.is_open()) {
      spdlog::error("Unable to open {} for output.", outputFilename);
      exit(-1);
    }
  }

  // If playing back an input file, run a different main loop ...
  if (!inputFilename.empty()) {
    spdlog::info("Playing back file {}", inputFilename);
    playbackSonarFile(inputFilename, output, stopAfter);
    return 0;
  }

  int count = 0;

#ifdef _WIN32
  signal(SIGINT, signalHandler);
#else
  signal(SIGHUP, signalHandler);
#endif

  spdlog::debug("Starting loop");

  SonarConfiguration config;
  config.setPingRate(pingRateNormal);

  spdlog::info("Setting range to {}", range);
  config.setRange(range);

  spdlog::info("Setting gain to {}", gain);
  config.setGainPercent(gain).noGainAssistance();

  if (bitDepth == 8) {
    config.setDataSize(dataSize8Bit);
  } else if (bitDepth == 16) {
    config.setDataSize(dataSize16Bit);
  } else if (bitDepth == 32) {
    config.sendGain().setDataSize(dataSize32Bit);
  }

  _io_thread.reset(new IoServiceThread);
  DataRx _data_rx(_io_thread->context());
  StatusRx _status_rx(_io_thread->context());

  // Callback for a SimplePingResultV1
  _data_rx.setCallback<liboculus::SimplePingResultV1>(
      [&](const liboculus::SimplePingResultV1 &ping) {
        spdlog::debug("PingV1: begin");
        // Pings are only sent to the callback if valid()
        // don't need to check independently

        {
          const auto valid = checkPingAgreesWithConfig(ping, config);
          if (!valid) {
            spdlog::warn("Mismatch between requested config and ping");
          }
        }

        std::vector<std::string> dump_vec;
        spdlog::debug("PingV1: dump");
        ping.dump(dump_vec);

        for (auto const &l : dump_vec) {
          spdlog::debug("PingV1: {}", l);
        }

        if (output.is_open()) {
          const char *cdata =
              reinterpret_cast<const char *>(ping.buffer()->data());
          output.write(cdata, ping.buffer()->size());
        }

        spdlog::debug("PingV1: mean");
        spdlog::info("Average intensity: {}",
                     mean_image_intensity(ping.image()));

        count++;
        if ((stopAfter > 0) && (count >= stopAfter))
          _io_thread->stop();
        spdlog::debug("PingV1: end");
      });

  // Callback for a SimplePingResultV2
  _data_rx.setCallback<liboculus::SimplePingResultV2>(
      [&](const liboculus::SimplePingResultV2 &ping) {
        spdlog::debug("PingV2: begin");
        // Pings are only sent to the callback if valid()
        // don't need to check independently

        {
          const auto valid = checkPingAgreesWithConfig(ping, config);
          if (!valid) {
            spdlog::warn("Mismatch between requested config and ping");
          }
        }

        std::vector<std::string> dump_vec;
        spdlog::debug("PingV2: dump");
        ping.dump(dump_vec);

        for (auto const &l : dump_vec) {
          spdlog::debug("PingV2: {}", l);
        }

        if (output.is_open()) {
          const char *cdata =
              reinterpret_cast<const char *>(ping.buffer()->data());
          output.write(cdata, ping.buffer()->size());
        }

        spdlog::debug("PingV2: mean");
        spdlog::debug("Average intensity: {}",
                      mean_image_intensity(ping.image()));

        count++;
        if ((stopAfter > 0) && (count >= stopAfter))
          doStop = true;
        spdlog::debug("PingV2: end");
      });

  // When the _data_rx connects, send the configuration
  _data_rx.setOnConnectCallback([&]() {
    std::vector<std::string> dump_vec;
    config.dump(dump_vec);

    for (auto const &l : dump_vec) {
      spdlog::debug("Config: {}", l);
    }

    _data_rx.sendSimpleFireMessage(config);
  });

  // Connect the client
  if (ipAddr == "auto") {
    // To autoconnect, define a callback for the _status_rx which
    // connects _data_rx to the received IP address
    _status_rx.setCallback([&](const SonarStatus &status, bool is_valid) {
      if (!is_valid || _data_rx.isConnected())
        return;
      _data_rx.connect(status.ipAddr());
    });
  } else {
    // Otherwise, just (attempt to) connect the DataRx to the specified IP
    // address
    _data_rx.connect(ipAddr);
  }
  _io_thread->start();

  int lastCount = 0;
  while (!doStop) {
    // Very rough Hz calculation right now
    const auto c = count;
    spdlog::info("Received pings at {} Hz", c - lastCount);

    lastCount = c;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  _io_thread->stop();
  _io_thread->join();

  if (output.is_open())
    output.close();

  spdlog::info("At exit");

  return 0;
}

int playbackSonarFile(const std::string &filename, ofstream &output,
                      int stopAfter) {
  shared_ptr<SonarPlayerBase> player(SonarPlayerBase::OpenFile(filename));

  if (!player) {
    spdlog::warn("Unable to open sonar file {}", filename);
    return -1;
  }

  if (!player->open(filename)) {
    spdlog::error("Failed to open ");
    return -1;
  }

  int count = 0;

  // Callback for a SimplePingResultV1
  player->setCallback<liboculus::SimplePingResultV1>(
      [&](const liboculus::SimplePingResultV1 &ping) {
        spdlog::debug("Playback V1: begin");
        // Pings are only sent to the callback if valid()
        // don't need to check independently

        std::vector<std::string> dump_vec;
        try {
          ping.dump(dump_vec);
        } catch (const std::exception &e) {
          spdlog::error("Playback V1: dump exception: {}", e.what());
          return;
        } catch (...) {
          spdlog::error("Playback V1: dump exception: unknown");
          return;
        }

        for (auto const &l : dump_vec) {
          spdlog::debug("PingV1: {}", l);
        }

        if (output.is_open()) {
          const char *cdata =
              reinterpret_cast<const char *>(ping.buffer()->data());
          output.write(cdata, ping.buffer()->size());
          spdlog::debug("Playback V1: wrote {} bytes",
                        ping.buffer()->size());
        }

        try {
          spdlog::info("Average intensity: {}",
                       mean_image_intensity(ping.image()));
        } catch (const std::exception &e) {
          spdlog::error("Playback V1: mean exception: {}", e.what());
          return;
        } catch (...) {
          spdlog::error("Playback V1: mean exception: unknown");
          return;
        }

        count++;
        spdlog::debug("Playback V1: end");
      });

  // Callback for a SimplePingResultV2
  player->setCallback<liboculus::SimplePingResultV2>(
      [&](const liboculus::SimplePingResultV2 &ping) {
        spdlog::debug("Playback V2: begin");
        // Pings are only sent to the callback if valid()
        // don't need to check independently

        std::vector<std::string> dump_vec;
        try {
          ping.dump(dump_vec);
        } catch (const std::exception &e) {
          spdlog::error("Playback V2: dump exception: {}", e.what());
          return;
        } catch (...) {
          spdlog::error("Playback V2: dump exception: unknown");
          return;
        }

        for (auto const &l : dump_vec) {
          spdlog::debug("PingV2: {}", l);
        }

        if (output.is_open()) {
          const char *cdata =
              reinterpret_cast<const char *>(ping.buffer()->data());
          output.write(cdata, ping.buffer()->size());
          spdlog::debug("Playback V2: wrote {} bytes",
                        ping.buffer()->size());
        }

        try {
          spdlog::info("Average intensity: {}",
                       mean_image_intensity(ping.image()));
        } catch (const std::exception &e) {
          spdlog::error("Playback V2: mean exception: {}", e.what());
          return;
        } catch (...) {
          spdlog::error("Playback V2: mean exception: unknown");
          return;
        }

        count++;
        spdlog::debug("Playback V2: end");
      });

  // SimplePingResult ping;
  while (player->nextPing() && !player->eof()) {
    spdlog::debug("Read a ping");
    ;
  }

  spdlog::info("{} sonar packets decoded", count);

  return 0;
}
