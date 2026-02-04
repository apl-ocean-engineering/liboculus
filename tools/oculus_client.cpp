#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

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
#include "liboculus/thirdparty/Oculus/Oculus.h"

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
namespace fs = std::filesystem;

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

void write_kv(std::ofstream &out, const std::string &key,
              const std::string &value) {
  out << key << "=" << value << "\n";
}

void write_kv(std::ofstream &out, const std::string &key, int value) {
  out << key << "=" << value << "\n";
}

void write_kv(std::ofstream &out, const std::string &key, uint32_t value) {
  out << key << "=" << value << "\n";
}

void write_kv(std::ofstream &out, const std::string &key, uint16_t value) {
  out << key << "=" << value << "\n";
}

void write_kv(std::ofstream &out, const std::string &key, double value) {
  out << key << "=" << value << "\n";
}

std::string trim_copy(const std::string &s) {
  const auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

std::unordered_map<std::string, std::string>
read_ini_kv(const std::string &path) {
  std::unordered_map<std::string, std::string> kv;
  std::ifstream in(path);
  if (!in.is_open()) {
    return kv;
  }
  std::string line;
  while (std::getline(in, line)) {
    const auto trimmed = trim_copy(line);
    if (trimmed.empty()) {
      continue;
    }
    if (trimmed[0] == '#' || trimmed[0] == ';') {
      continue;
    }
    if (trimmed[0] == '[') {
      continue;
    }
    const auto eq = trimmed.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const auto key = trim_copy(trimmed.substr(0, eq));
    const auto val = trim_copy(trimmed.substr(eq + 1));
    if (!key.empty()) {
      kv[key] = val;
    }
  }
  return kv;
}

bool write_ini_kv(const std::string &path,
                  const std::unordered_map<std::string, std::string> &kv,
                  std::string &error) {
  std::ofstream out(path);
  if (!out.is_open()) {
    error = "Unable to open profile for writing";
    return false;
  }
  out << "; occlient profile\n";
  out << "[occlient]\n";
  for (const auto &it : kv) {
    out << it.first << "=" << it.second << "\n";
  }
  if (!out) {
    error = "Failed while writing profile";
    return false;
  }
  return true;
}

uint32_t adler32(const uint8_t *data, size_t len) {
  const uint32_t mod_adler = 65521;
  uint32_t a = 1;
  uint32_t b = 0;
  for (size_t i = 0; i < len; ++i) {
    a = (a + data[i]) % mod_adler;
    b = (b + a) % mod_adler;
  }
  return (b << 16) | a;
}

uint32_t crc32(const uint8_t *data, size_t len) {
  static bool initialized = false;
  static uint32_t table[256];
  if (!initialized) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) {
        if (c & 1) {
          c = 0xEDB88320u ^ (c >> 1);
        } else {
          c >>= 1;
        }
      }
      table[i] = c;
    }
    initialized = true;
  }

  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    c = table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
  }
  return c ^ 0xFFFFFFFFu;
}

void write_u32_be(std::ofstream &out, uint32_t v) {
  const uint8_t b[4] = {
      static_cast<uint8_t>((v >> 24) & 0xFF),
      static_cast<uint8_t>((v >> 16) & 0xFF),
      static_cast<uint8_t>((v >> 8) & 0xFF),
      static_cast<uint8_t>(v & 0xFF)};
  out.write(reinterpret_cast<const char *>(b), 4);
}

bool write_chunk(std::ofstream &out, const char type[4],
                 const std::vector<uint8_t> &data, std::string &error) {
  write_u32_be(out, static_cast<uint32_t>(data.size()));
  out.write(type, 4);
  if (!data.empty()) {
    out.write(reinterpret_cast<const char *>(data.data()),
              static_cast<std::streamsize>(data.size()));
  }

  std::vector<uint8_t> crc_input(4 + data.size());
  crc_input[0] = static_cast<uint8_t>(type[0]);
  crc_input[1] = static_cast<uint8_t>(type[1]);
  crc_input[2] = static_cast<uint8_t>(type[2]);
  crc_input[3] = static_cast<uint8_t>(type[3]);
  if (!data.empty()) {
    std::copy(data.begin(), data.end(), crc_input.begin() + 4);
  }

  const uint32_t crc = crc32(crc_input.data(), crc_input.size());
  write_u32_be(out, crc);
  if (!out) {
    error = "Failed while writing PNG chunk";
    return false;
  }
  return true;
}

bool write_png(const liboculus::ImageData &image, uint32_t max_override,
               const std::string &path, std::string &error) {
  const uint32_t width = static_cast<uint32_t>(image.nBeams());
  const uint32_t height = static_cast<uint32_t>(image.nRanges());
  if (width == 0 || height == 0) {
    error = "Image has zero width or height";
    return false;
  }

  uint32_t max_val = max_override;
  if (max_val == 0) {
    max_val = 0;
    for (uint32_t r = 0; r < height; ++r) {
      for (uint32_t b = 0; b < width; ++b) {
        max_val = std::max(max_val, image.at_uint32(b, r));
      }
    }
  }

  const size_t total = static_cast<size_t>(width) * height;
  std::vector<uint8_t> pixels(total, 0);
  if (max_val > 0) {
    for (uint32_t r = 0; r < height; ++r) {
      for (uint32_t b = 0; b < width; ++b) {
        const uint32_t v = image.at_uint32(b, r);
        const uint64_t scaled = (static_cast<uint64_t>(v) * 255u) / max_val;
        pixels[static_cast<size_t>(r) * width + b] =
            static_cast<uint8_t>(std::min<uint64_t>(scaled, 255u));
      }
    }
  }

  std::ofstream out(path, std::ios::binary);
  if (!out.is_open()) {
    error = "Unable to open output file";
    return false;
  }

  const uint8_t signature[8] = {0x89, 'P', 'N', 'G',
                                0x0D, 0x0A, 0x1A, 0x0A};
  out.write(reinterpret_cast<const char *>(signature), 8);

  std::vector<uint8_t> ihdr(13, 0);
  ihdr[0] = static_cast<uint8_t>((width >> 24) & 0xFF);
  ihdr[1] = static_cast<uint8_t>((width >> 16) & 0xFF);
  ihdr[2] = static_cast<uint8_t>((width >> 8) & 0xFF);
  ihdr[3] = static_cast<uint8_t>(width & 0xFF);
  ihdr[4] = static_cast<uint8_t>((height >> 24) & 0xFF);
  ihdr[5] = static_cast<uint8_t>((height >> 16) & 0xFF);
  ihdr[6] = static_cast<uint8_t>((height >> 8) & 0xFF);
  ihdr[7] = static_cast<uint8_t>(height & 0xFF);
  ihdr[8] = 8;
  ihdr[9] = 0;
  ihdr[10] = 0;
  ihdr[11] = 0;
  ihdr[12] = 0;

  if (!write_chunk(out, "IHDR", ihdr, error)) {
    return false;
  }

  std::vector<uint8_t> raw;
  raw.reserve(height * (width + 1));
  for (uint32_t r = 0; r < height; ++r) {
    raw.push_back(0);
    const size_t row_offset = static_cast<size_t>(r) * width;
    raw.insert(raw.end(), pixels.begin() + row_offset,
               pixels.begin() + row_offset + width);
  }

  std::vector<uint8_t> zlib;
  zlib.reserve(raw.size() + 64);
  zlib.push_back(0x78);
  zlib.push_back(0x01);

  size_t pos = 0;
  while (pos < raw.size()) {
    const size_t remaining = raw.size() - pos;
    const uint16_t block_len =
        static_cast<uint16_t>(std::min<size_t>(remaining, 65535));
    const bool is_final = (pos + block_len) == raw.size();
    zlib.push_back(static_cast<uint8_t>(is_final ? 0x01 : 0x00));
    zlib.push_back(static_cast<uint8_t>(block_len & 0xFF));
    zlib.push_back(static_cast<uint8_t>((block_len >> 8) & 0xFF));
    const uint16_t nlen = static_cast<uint16_t>(~block_len);
    zlib.push_back(static_cast<uint8_t>(nlen & 0xFF));
    zlib.push_back(static_cast<uint8_t>((nlen >> 8) & 0xFF));

    zlib.insert(zlib.end(), raw.begin() + pos, raw.begin() + pos + block_len);
    pos += block_len;
  }

  const uint32_t adler = adler32(raw.data(), raw.size());
  zlib.push_back(static_cast<uint8_t>((adler >> 24) & 0xFF));
  zlib.push_back(static_cast<uint8_t>((adler >> 16) & 0xFF));
  zlib.push_back(static_cast<uint8_t>((adler >> 8) & 0xFF));
  zlib.push_back(static_cast<uint8_t>(adler & 0xFF));

  if (!write_chunk(out, "IDAT", zlib, error)) {
    return false;
  }

  const std::vector<uint8_t> empty;
  if (!write_chunk(out, "IEND", empty, error)) {
    return false;
  }

  return true;
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
  app.add_option("--ip", ipAddr,
                 "IP address of sonar or \"auto\" to automatically detect.");
  app.allow_extras();

  string outputFilename("");
  app.add_option("-o,--output", outputFilename,
                 "Saves raw sonar data to specified file.");

  // Playback currently not working
  string inputFilename("");
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

  int beams = 512;
  app.add_option("--beams", beams, "Number of beams (256 or 512)");

  bool noGain = false;
  app.add_flag("--no-gain", noGain, "Do not include gain data in returns");

  bool fullReturn = false;
  app.add_flag("--full-return", fullReturn, "Request full return messages");

  bool forceV1 = false;
  app.add_flag("--fire-v1", forceV1, "Send OculusSimpleFireMessage v1");

  bool printStatus = false;
  app.add_flag("--print-status", printStatus,
               "Print key fields from status packets");

  bool statsOnly = false;
  app.add_flag("--stats", statsOnly, "Only print FPS stats");

  bool fireDump = false;
  app.add_flag("--dump-fire", fireDump,
               "Print FireMessage bytes and key fields");

  bool compareFire = false;
  app.add_flag("--compare-fire", compareFire,
               "Compare ping fields with last FireMessage");

  string videoDir("");
  app.add_option("--video-dir", videoDir,
                 "Directory to write PNG frames (e.g. out_frames)");

  uint32_t videoMax = 0;
  app.add_option("--video-max", videoMax,
                 "Max value for PNG scaling (0 = auto)");

  bool profileLoad = false;
  app.add_flag("--profile-load", profileLoad,
               "Load settings from occlient.ini");
  bool profileSave = false;
  app.add_flag("--profile-save", profileSave,
               "Save settings to occlient.ini and exit");

  string fieldsPath("");
  app.add_option("--dump-fields", fieldsPath,
                 "Write parsed ping fields to a text file");

  CLI11_PARSE(app, argc, argv);

  if (ipAddr == "auto") {
    const auto extras = app.remaining();
    if (!extras.empty()) {
      ipAddr = extras.front();
    }
  }

  const std::string profilePath = "occlient.ini";
  if (profileLoad) {
    const auto kv = read_ini_kv(profilePath);
    if (!kv.empty()) {
      if (kv.count("ip")) ipAddr = kv.at("ip");
      if (kv.count("range")) range = std::stof(kv.at("range"));
      if (kv.count("gain")) gain = std::stof(kv.at("gain"));
      if (kv.count("bits")) bitDepth = std::stoi(kv.at("bits"));
      if (kv.count("beams")) beams = std::stoi(kv.at("beams"));
      if (kv.count("no_gain")) noGain = (kv.at("no_gain") == "1");
      if (kv.count("full_return")) fullReturn = (kv.at("full_return") == "1");
      if (kv.count("fire_v1")) forceV1 = (kv.at("fire_v1") == "1");
      if (kv.count("print_status"))
        printStatus = (kv.at("print_status") == "1");
      if (kv.count("stats")) statsOnly = (kv.at("stats") == "1");
      if (kv.count("video_dir")) videoDir = kv.at("video_dir");
      if (kv.count("video_max")) videoMax = std::stoul(kv.at("video_max"));
      if (kv.count("output")) outputFilename = kv.at("output");
      if (kv.count("frames")) stopAfter = std::stoi(kv.at("frames"));
    }
  }

  if (profileSave) {
    std::unordered_map<std::string, std::string> kv;
    kv["ip"] = ipAddr;
    kv["range"] = std::to_string(range);
    kv["gain"] = std::to_string(gain);
    kv["bits"] = std::to_string(bitDepth);
    kv["beams"] = std::to_string(beams);
    kv["no_gain"] = noGain ? "1" : "0";
    kv["full_return"] = fullReturn ? "1" : "0";
    kv["fire_v1"] = forceV1 ? "1" : "0";
    kv["print_status"] = printStatus ? "1" : "0";
    kv["stats"] = statsOnly ? "1" : "0";
    kv["video_dir"] = videoDir;
    kv["video_max"] = std::to_string(videoMax);
    kv["output"] = outputFilename;
    kv["frames"] = std::to_string(stopAfter);

    std::string error;
    if (!write_ini_kv(profilePath, kv, error)) {
      spdlog::error("Profile save failed: {}", error);
      return 1;
    }
    spdlog::info("Saved profile to {}", profilePath);
    return 0;
  }

  if (verbosity == 1) {
    spdlog::set_level(spdlog::level::debug);
  } else if (verbosity > 1) {
    spdlog::set_level(spdlog::level::trace);
  }
  if (statsOnly) {
    spdlog::set_level(spdlog::level::warn);
  }

  if ((bitDepth != 8) && (bitDepth != 16) && (bitDepth != 32)) {
    spdlog::error("Invalid bit depth {}", bitDepth);
    exit(-1);
  }

  if ((gain < 1) || (gain > 100)) {
    spdlog::error("Invalid gain {}; should be in the range of 1-100", gain);
  }

  if ((beams != 256) && (beams != 512)) {
    spdlog::error("Invalid beams {}; should be 256 or 512", beams);
    exit(-1);
  }

  ofstream output;
  ofstream fieldsOut;
  std::mutex fieldsMutex;

  if (!outputFilename.empty()) {
    spdlog::debug("Opening output file {}", outputFilename);
    output.open(outputFilename, ios_base::binary | ios_base::out);

    if (!output.is_open()) {
      spdlog::error("Unable to open {} for output.", outputFilename);
      exit(-1);
    }
  }
  if (!videoDir.empty()) {
    std::error_code ec;
    fs::create_directories(videoDir, ec);
    if (ec) {
      spdlog::error("Unable to create video dir {}: {}", videoDir,
                    ec.message());
      exit(-1);
    }
  }

  if (!fieldsPath.empty()) {
    fieldsOut.open(fieldsPath, ios_base::out | ios_base::app);
    if (!fieldsOut.is_open()) {
      spdlog::error("Unable to open {} for field output.", fieldsPath);
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
  int frameIndex = 0;

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

  if (beams == 512) {
    config.use512Beams();
  } else {
    config.use256Beams();
  }

  const bool sendGain = !noGain;
  if (sendGain) {
    config.sendGain();
  } else {
    config.dontSendGain();
  }

  const bool simpleReturn = !fullReturn;
  if (simpleReturn) {
    config.setSimpleReturn(true);
  } else {
    config.setSimpleReturn(false);
  }

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
        if (!statsOnly) {
          spdlog::debug("PingV1: begin");
        }
        // Pings are only sent to the callback if valid()
        // don't need to check independently

        {
          const auto valid = checkPingAgreesWithConfig(ping, config);
          if (!valid) {
            spdlog::warn("Mismatch between requested config and ping");
          }
        }

        if (!statsOnly) {
          std::vector<std::string> dump_vec;
          spdlog::debug("PingV1: dump");
          ping.dump(dump_vec);

          for (auto const &l : dump_vec) {
            spdlog::debug("PingV1: {}", l);
          }
        }

        if (output.is_open()) {
          const char *cdata =
              reinterpret_cast<const char *>(ping.buffer()->data());
          output.write(cdata, ping.buffer()->size());
        }

        if (!statsOnly) {
          spdlog::debug("PingV1: mean");
          spdlog::info("Average intensity: {}",
                       mean_image_intensity(ping.image()));
        }

        if (compareFire) {
          spdlog::info(
              "PingV1 vs Fire: range={}m gain={} fireFlags=0x{:02x} dataSize={} beams={} simpleReturn={}",
              ping.fireMsg()->range, ping.fireMsg()->gainPercent,
              static_cast<int>(ping.fireMsg()->flags),
              static_cast<int>(ping.ping()->dataSize), ping.ping()->nBeams,
              ping.flags().getSimpleReturn() ? "true" : "false");
        }

        if (fieldsOut.is_open()) {
          std::lock_guard<std::mutex> lock(fieldsMutex);
          fieldsOut << "ping_version=1\n";
          write_kv(fieldsOut, "msg_id",
                   static_cast<int>(ping.hdr()->msgId));
          write_kv(fieldsOut, "msg_version", ping.hdr()->msgVersion);
          write_kv(fieldsOut, "src_id", ping.hdr()->srcDeviceId);
          write_kv(fieldsOut, "dst_id", ping.hdr()->dstDeviceId);
          write_kv(fieldsOut, "payload_size", ping.hdr()->payloadSize);
          write_kv(fieldsOut, "range", ping.fireMsg()->range);
          write_kv(fieldsOut, "gain", ping.fireMsg()->gainPercent);
          write_kv(fieldsOut, "frequency", ping.ping()->frequency);
          write_kv(fieldsOut, "temperature", ping.ping()->temperature);
          write_kv(fieldsOut, "pressure", ping.ping()->pressure);
          write_kv(fieldsOut, "speed_of_sound", ping.ping()->speedOfSoundUsed);
          write_kv(fieldsOut, "range_resolution", ping.ping()->rangeResolution);
          write_kv(fieldsOut, "n_ranges", ping.ping()->nRanges);
          write_kv(fieldsOut, "n_beams", ping.ping()->nBeams);
          write_kv(fieldsOut, "image_offset", ping.ping()->imageOffset);
          write_kv(fieldsOut, "image_size", ping.ping()->imageSize);
          write_kv(fieldsOut, "message_size", ping.ping()->messageSize);
          fieldsOut << "\n";
        }

        if (!videoDir.empty()) {
          std::string error;
          const auto filename =
              (fs::path(videoDir) /
               fmt::format("frame_{:06d}.png", frameIndex))
                  .string();
          if (!write_png(ping.image(), videoMax, filename, error)) {
            spdlog::error("Failed to write PNG {}: {}", filename, error);
          } else if (!statsOnly) {
            spdlog::debug("Wrote {}", filename);
          }
          frameIndex++;
        }

        count++;
        if ((stopAfter > 0) && (count >= stopAfter))
          _io_thread->stop();
        if (!statsOnly) {
          spdlog::debug("PingV1: end");
        }
      });

  // Callback for a SimplePingResultV2
  _data_rx.setCallback<liboculus::SimplePingResultV2>(
      [&](const liboculus::SimplePingResultV2 &ping) {
        if (!statsOnly) {
          spdlog::debug("PingV2: begin");
        }
        // Pings are only sent to the callback if valid()
        // don't need to check independently

        {
          const auto valid = checkPingAgreesWithConfig(ping, config);
          if (!valid) {
            spdlog::warn("Mismatch between requested config and ping");
          }
        }

        if (!statsOnly) {
          std::vector<std::string> dump_vec;
          spdlog::debug("PingV2: dump");
          ping.dump(dump_vec);

          for (auto const &l : dump_vec) {
            spdlog::debug("PingV2: {}", l);
          }
        }

        if (output.is_open()) {
          const char *cdata =
              reinterpret_cast<const char *>(ping.buffer()->data());
          output.write(cdata, ping.buffer()->size());
        }

        if (!statsOnly) {
          spdlog::debug("PingV2: mean");
          spdlog::debug("Average intensity: {}",
                        mean_image_intensity(ping.image()));
        }

        if (compareFire) {
          spdlog::info(
              "PingV2 vs Fire: range={}m gain={} fireFlags=0x{:02x} dataSize={} beams={} simpleReturn={}",
              ping.fireMsg()->rangePercent, ping.fireMsg()->gainPercent,
              static_cast<int>(ping.fireMsg()->flags),
              static_cast<int>(ping.ping()->dataSize), ping.ping()->nBeams,
              ping.flags().getSimpleReturn() ? "true" : "false");
        }

        if (fieldsOut.is_open()) {
          std::lock_guard<std::mutex> lock(fieldsMutex);
          fieldsOut << "ping_version=2\n";
          write_kv(fieldsOut, "msg_id",
                   static_cast<int>(ping.hdr()->msgId));
          write_kv(fieldsOut, "msg_version", ping.hdr()->msgVersion);
          write_kv(fieldsOut, "src_id", ping.hdr()->srcDeviceId);
          write_kv(fieldsOut, "dst_id", ping.hdr()->dstDeviceId);
          write_kv(fieldsOut, "payload_size", ping.hdr()->payloadSize);
          write_kv(fieldsOut, "range", ping.fireMsg()->rangePercent);
          write_kv(fieldsOut, "gain", ping.fireMsg()->gainPercent);
          write_kv(fieldsOut, "frequency", ping.ping()->frequency);
          write_kv(fieldsOut, "temperature", ping.ping()->temperature);
          write_kv(fieldsOut, "pressure", ping.ping()->pressure);
          write_kv(fieldsOut, "heading", ping.ping()->heading);
          write_kv(fieldsOut, "pitch", ping.ping()->pitch);
          write_kv(fieldsOut, "roll", ping.ping()->roll);
          write_kv(fieldsOut, "speed_of_sound", ping.ping()->speedOfSoundUsed);
          write_kv(fieldsOut, "range_resolution", ping.ping()->rangeResolution);
          write_kv(fieldsOut, "n_ranges", ping.ping()->nRanges);
          write_kv(fieldsOut, "n_beams", ping.ping()->nBeams);
          write_kv(fieldsOut, "image_offset", ping.ping()->imageOffset);
          write_kv(fieldsOut, "image_size", ping.ping()->imageSize);
          write_kv(fieldsOut, "message_size", ping.ping()->messageSize);
          fieldsOut << "\n";
        }

        if (!videoDir.empty()) {
          std::string error;
          const auto filename =
              (fs::path(videoDir) /
               fmt::format("frame_{:06d}.png", frameIndex))
                  .string();
          if (!write_png(ping.image(), videoMax, filename, error)) {
            spdlog::error("Failed to write PNG {}: {}", filename, error);
          } else if (!statsOnly) {
            spdlog::debug("Wrote {}", filename);
          }
          frameIndex++;
        }

        count++;
        if ((stopAfter > 0) && (count >= stopAfter))
          doStop = true;
        if (!statsOnly) {
          spdlog::debug("PingV2: end");
        }
      });

  // When the _data_rx connects, send the configuration
  _data_rx.setOnConnectCallback([&]() {
    if (!statsOnly) {
      std::vector<std::string> dump_vec;
      config.dump(dump_vec);

      for (auto const &l : dump_vec) {
        spdlog::debug("Config: {}", l);
      }
    }

    if (fireDump) {
      std::vector<uint8_t> raw;
      if (forceV1) {
        raw = config.serialize<OculusSimpleFireMessage>();
      } else {
        raw = config.serialize<OculusSimpleFireMessage2>();
      }

      std::ostringstream hex;
      hex << std::hex << std::setfill('0');
      for (size_t i = 0; i < raw.size(); ++i) {
        hex << std::setw(2) << static_cast<int>(raw[i]);
        if ((i + 1) % 16 == 0) {
          hex << "\n";
        } else {
          hex << " ";
        }
      }
      spdlog::info("FireMessage bytes ({}):\n{}", raw.size(), hex.str());

      if (forceV1 && raw.size() >= sizeof(OculusSimpleFireMessage)) {
        auto msg =
            reinterpret_cast<const OculusSimpleFireMessage *>(raw.data());
        spdlog::info(
            "FireMessage v1 fields: mode={} pingRate={} netSpeed={} gamma={} "
            "flags=0x{:02x} range={} gain={} sos={} salinity={}",
            static_cast<int>(msg->masterMode),
            static_cast<int>(msg->pingRate),
            static_cast<int>(msg->networkSpeed),
            static_cast<int>(msg->gammaCorrection),
            static_cast<int>(msg->flags), msg->range, msg->gainPercent,
            msg->speedOfSound, msg->salinity);
      } else if (!forceV1 &&
                 raw.size() >= sizeof(OculusSimpleFireMessage2)) {
        auto msg =
            reinterpret_cast<const OculusSimpleFireMessage2 *>(raw.data());
        spdlog::info(
            "FireMessage v2 fields: mode={} pingRate={} netSpeed={} gamma={} "
            "flags=0x{:02x} rangePct={} gain={} sos={} salinity={} ext=0x{:08x}",
            static_cast<int>(msg->masterMode),
            static_cast<int>(msg->pingRate),
            static_cast<int>(msg->networkSpeed),
            static_cast<int>(msg->gammaCorrection),
            static_cast<int>(msg->flags), msg->rangePercent,
            msg->gainPercent, msg->speedOfSound, msg->salinity, msg->extFlags);
      }
    }

    if (forceV1) {
      spdlog::info("Sending FireMessage v1");
      _data_rx.sendSimpleFireMessage<OculusSimpleFireMessage>(config);
    } else {
      spdlog::info("Sending FireMessage v2");
      _data_rx.sendSimpleFireMessage(config);
    }
  });

  // Connect the client
  _status_rx.setCallback([&](const SonarStatus &status, bool is_valid) {
    if (printStatus) {
      std::vector<std::string> dump_vec;
      status.dump(dump_vec);
      for (auto const &l : dump_vec) {
        spdlog::info("Status: {}", l);
      }
    }

    if (ipAddr != "auto") {
      return;
    }

    if (!is_valid || _data_rx.isConnected()) {
      return;
    }

    _data_rx.connect(status.ipAddr());
  });

  if (ipAddr != "auto") {
    // Otherwise, just (attempt to) connect the DataRx to the specified IP
    // address
    _data_rx.connect(ipAddr);
  }
  _io_thread->start();

  int lastCount = 0;
  while (!doStop) {
    // Very rough Hz calculation right now
    const auto c = count;
    if (statsOnly) {
      std::cout << "FPS: " << (c - lastCount) << std::endl;
    } else {
      spdlog::info("Received pings at {} Hz", c - lastCount);
    }

    lastCount = c;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  _io_thread->stop();
  _io_thread->join();

  if (output.is_open())
    output.close();
  if (fieldsOut.is_open())
    fieldsOut.close();

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
