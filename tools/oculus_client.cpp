#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string_view>
#include <type_traits>
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
// Make these global so signal handler can access it
std::unique_ptr<liboculus::IoServiceThread> _io_thread;
std::atomic<bool> doStop{false};



// Catch signals
void signalHandler(int signo) {
  if (_io_thread)
    _io_thread->stop();
  doStop.store(true, std::memory_order_relaxed);
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

template <typename T,
          typename = std::enable_if_t<std::is_arithmetic_v<T> ||
                                      std::is_convertible_v<T, std::string_view>>>
void write_kv(std::ofstream &out, const std::string &key, T value) {
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

struct AppOptions {
  int verbosity = 0;
  std::string ipAddr = "auto";
  std::string outputFilename;
  std::string inputFilename;
  int bitDepth = 8;
  int stopAfter = -1;
  float range = 4;
  float gain = 50;
  int beams = 512;
  bool noGain = false;
  bool fullReturn = false;
  bool forceV1 = false;
  bool printStatus = false;
  bool statsOnly = false;
  bool fireDump = false;
  bool compareFire = false;
  std::string videoDir;
  uint32_t videoMax = 0;
  bool profileLoad = false;
  bool profileSave = false;
  std::string fieldsPath;
};

// Shared state used by callbacks and the main loop.
struct RuntimeContext {
  std::atomic<int> count{0};
  int frameIndex = 0;
  std::ofstream *output = nullptr;
  std::ofstream *fieldsOut = nullptr;
  std::mutex *fieldsMutex = nullptr;
  bool computeMean = false;
};

void configure_logging(const AppOptions &opt) {
  if (opt.verbosity == 1) {
    spdlog::set_level(spdlog::level::debug);
  } else if (opt.verbosity > 1) {
    spdlog::set_level(spdlog::level::trace);
  }
  if (opt.statsOnly) {
    spdlog::set_level(spdlog::level::warn);
  }
}

bool validate_options(const AppOptions &opt) {
  if ((opt.bitDepth != 8) && (opt.bitDepth != 16) && (opt.bitDepth != 32)) {
    spdlog::error("Invalid bit depth {}", opt.bitDepth);
    return false;
  }

  if ((opt.gain < 1) || (opt.gain > 100)) {
    spdlog::error("Invalid gain {}; should be in the range of 1-100", opt.gain);
    return false;
  }

  if ((opt.beams != 256) && (opt.beams != 512)) {
    spdlog::error("Invalid beams {}; should be 256 or 512", opt.beams);
    return false;
  }

  return true;
}

void apply_profile_kv(const std::unordered_map<std::string, std::string> &kv,
                      AppOptions &opt) {
  if (kv.count("ip")) opt.ipAddr = kv.at("ip");
  if (kv.count("range")) opt.range = std::stof(kv.at("range"));
  if (kv.count("gain")) opt.gain = std::stof(kv.at("gain"));
  if (kv.count("bits")) opt.bitDepth = std::stoi(kv.at("bits"));
  if (kv.count("beams")) opt.beams = std::stoi(kv.at("beams"));
  if (kv.count("no_gain")) opt.noGain = (kv.at("no_gain") == "1");
  if (kv.count("full_return")) opt.fullReturn = (kv.at("full_return") == "1");
  if (kv.count("fire_v1")) opt.forceV1 = (kv.at("fire_v1") == "1");
  if (kv.count("print_status"))
    opt.printStatus = (kv.at("print_status") == "1");
  if (kv.count("stats")) opt.statsOnly = (kv.at("stats") == "1");
  if (kv.count("video_dir")) opt.videoDir = kv.at("video_dir");
  if (kv.count("video_max")) opt.videoMax = std::stoul(kv.at("video_max"));
  if (kv.count("output")) opt.outputFilename = kv.at("output");
  if (kv.count("frames")) opt.stopAfter = std::stoi(kv.at("frames"));
}

bool save_profile(const AppOptions &opt, const std::string &profilePath) {
  std::unordered_map<std::string, std::string> kv;
  kv["ip"] = opt.ipAddr;
  kv["range"] = std::to_string(opt.range);
  kv["gain"] = std::to_string(opt.gain);
  kv["bits"] = std::to_string(opt.bitDepth);
  kv["beams"] = std::to_string(opt.beams);
  kv["no_gain"] = opt.noGain ? "1" : "0";
  kv["full_return"] = opt.fullReturn ? "1" : "0";
  kv["fire_v1"] = opt.forceV1 ? "1" : "0";
  kv["print_status"] = opt.printStatus ? "1" : "0";
  kv["stats"] = opt.statsOnly ? "1" : "0";
  kv["video_dir"] = opt.videoDir;
  kv["video_max"] = std::to_string(opt.videoMax);
  kv["output"] = opt.outputFilename;
  kv["frames"] = std::to_string(opt.stopAfter);

  std::string error;
  if (!write_ini_kv(profilePath, kv, error)) {
    spdlog::error("Profile save failed: {}", error);
    return false;
  }
  spdlog::info("Saved profile to {}", profilePath);
  return true;
}

bool open_outputs(const AppOptions &opt, std::ofstream &output,
                  std::ofstream &fieldsOut) {
  if (!opt.outputFilename.empty()) {
    spdlog::debug("Opening output file {}", opt.outputFilename);
    output.open(opt.outputFilename, ios_base::binary | ios_base::out);

    if (!output.is_open()) {
      spdlog::error("Unable to open {} for output.", opt.outputFilename);
      return false;
    }
  }
  if (!opt.videoDir.empty()) {
    std::error_code ec;
    fs::create_directories(opt.videoDir, ec);
    if (ec) {
      spdlog::error("Unable to create video dir {}: {}", opt.videoDir,
                    ec.message());
      return false;
    }
  }

  if (!opt.fieldsPath.empty()) {
    fieldsOut.open(opt.fieldsPath, ios_base::out | ios_base::app);
    if (!fieldsOut.is_open()) {
      spdlog::error("Unable to open {} for field output.", opt.fieldsPath);
      return false;
    }
  }
  return true;
}

SonarConfiguration build_config(const AppOptions &opt) {
  SonarConfiguration config;
  config.setPingRate(pingRateNormal);
  config.setRange(opt.range);
  config.setGainPercent(opt.gain).noGainAssistance();

  if (opt.beams == 512) {
    config.use512Beams();
  } else {
    config.use256Beams();
  }

  const bool sendGain = !opt.noGain;
  if (sendGain) {
    config.sendGain();
  } else {
    config.dontSendGain();
  }

  const bool simpleReturn = !opt.fullReturn;
  config.setSimpleReturn(simpleReturn);

  if (opt.bitDepth == 8) {
    config.setDataSize(dataSize8Bit);
  } else if (opt.bitDepth == 16) {
    config.setDataSize(dataSize16Bit);
  } else if (opt.bitDepth == 32) {
    config.sendGain().setDataSize(dataSize32Bit);
  }

  return config;
}

void handle_frame_stop(const AppOptions &opt, std::atomic<int> &count) {
  // Enforce stop-after across both ping versions.
  const int newCount = count.fetch_add(1, std::memory_order_relaxed) + 1;
  if ((opt.stopAfter > 0) && (newCount >= opt.stopAfter)) {
    doStop.store(true, std::memory_order_relaxed);
    if (_io_thread) {
      _io_thread->stop();
    }
  }
}

void write_png_frame(const AppOptions &opt, RuntimeContext &ctx,
                     const liboculus::ImageData &image) {
  if (opt.videoDir.empty()) {
    return;
  }
  std::string error;
  const auto filename =
      (fs::path(opt.videoDir) / fmt::format("frame_{:06d}.png", ctx.frameIndex))
          .string();
  if (!write_png(image, opt.videoMax, filename, error)) {
    spdlog::error("Failed to write PNG {}: {}", filename, error);
  } else if (!opt.statsOnly) {
    spdlog::debug("Wrote {}", filename);
  }
  ctx.frameIndex++;
}

void write_fields_v1(RuntimeContext &ctx,
                     const liboculus::SimplePingResultV1 &ping) {
  if (!ctx.fieldsOut || !ctx.fieldsOut->is_open() || !ctx.fieldsMutex) {
    return;
  }
  std::lock_guard<std::mutex> lock(*ctx.fieldsMutex);
  (*ctx.fieldsOut) << "ping_version=1\n";
  write_kv(*ctx.fieldsOut, "msg_id", static_cast<int>(ping.hdr()->msgId));
  write_kv(*ctx.fieldsOut, "msg_version", ping.hdr()->msgVersion);
  write_kv(*ctx.fieldsOut, "src_id", ping.hdr()->srcDeviceId);
  write_kv(*ctx.fieldsOut, "dst_id", ping.hdr()->dstDeviceId);
  write_kv(*ctx.fieldsOut, "payload_size", ping.hdr()->payloadSize);
  write_kv(*ctx.fieldsOut, "range", ping.fireMsg()->range);
  write_kv(*ctx.fieldsOut, "gain", ping.fireMsg()->gainPercent);
  write_kv(*ctx.fieldsOut, "frequency", ping.ping()->frequency);
  write_kv(*ctx.fieldsOut, "temperature", ping.ping()->temperature);
  write_kv(*ctx.fieldsOut, "pressure", ping.ping()->pressure);
  write_kv(*ctx.fieldsOut, "speed_of_sound", ping.ping()->speedOfSoundUsed);
  write_kv(*ctx.fieldsOut, "range_resolution", ping.ping()->rangeResolution);
  write_kv(*ctx.fieldsOut, "n_ranges", ping.ping()->nRanges);
  write_kv(*ctx.fieldsOut, "n_beams", ping.ping()->nBeams);
  write_kv(*ctx.fieldsOut, "image_offset", ping.ping()->imageOffset);
  write_kv(*ctx.fieldsOut, "image_size", ping.ping()->imageSize);
  write_kv(*ctx.fieldsOut, "message_size", ping.ping()->messageSize);
  (*ctx.fieldsOut) << "\n";
}

void write_fields_v2(RuntimeContext &ctx,
                     const liboculus::SimplePingResultV2 &ping) {
  if (!ctx.fieldsOut || !ctx.fieldsOut->is_open() || !ctx.fieldsMutex) {
    return;
  }
  std::lock_guard<std::mutex> lock(*ctx.fieldsMutex);
  (*ctx.fieldsOut) << "ping_version=2\n";
  write_kv(*ctx.fieldsOut, "msg_id", static_cast<int>(ping.hdr()->msgId));
  write_kv(*ctx.fieldsOut, "msg_version", ping.hdr()->msgVersion);
  write_kv(*ctx.fieldsOut, "src_id", ping.hdr()->srcDeviceId);
  write_kv(*ctx.fieldsOut, "dst_id", ping.hdr()->dstDeviceId);
  write_kv(*ctx.fieldsOut, "payload_size", ping.hdr()->payloadSize);
  write_kv(*ctx.fieldsOut, "range", ping.fireMsg()->rangePercent);
  write_kv(*ctx.fieldsOut, "gain", ping.fireMsg()->gainPercent);
  write_kv(*ctx.fieldsOut, "frequency", ping.ping()->frequency);
  write_kv(*ctx.fieldsOut, "temperature", ping.ping()->temperature);
  write_kv(*ctx.fieldsOut, "pressure", ping.ping()->pressure);
  write_kv(*ctx.fieldsOut, "heading", ping.ping()->heading);
  write_kv(*ctx.fieldsOut, "pitch", ping.ping()->pitch);
  write_kv(*ctx.fieldsOut, "roll", ping.ping()->roll);
  write_kv(*ctx.fieldsOut, "speed_of_sound", ping.ping()->speedOfSoundUsed);
  write_kv(*ctx.fieldsOut, "range_resolution", ping.ping()->rangeResolution);
  write_kv(*ctx.fieldsOut, "n_ranges", ping.ping()->nRanges);
  write_kv(*ctx.fieldsOut, "n_beams", ping.ping()->nBeams);
  write_kv(*ctx.fieldsOut, "image_offset", ping.ping()->imageOffset);
  write_kv(*ctx.fieldsOut, "image_size", ping.ping()->imageSize);
  write_kv(*ctx.fieldsOut, "message_size", ping.ping()->messageSize);
  (*ctx.fieldsOut) << "\n";
}

void dump_fire_message(const SonarConfiguration &config, bool forceV1) {
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
    auto msg = reinterpret_cast<const OculusSimpleFireMessage *>(raw.data());
    spdlog::info(
        "FireMessage v1 fields: mode={} pingRate={} netSpeed={} gamma={} "
        "flags=0x{:02x} range={} gain={} sos={} salinity={}",
        static_cast<int>(msg->masterMode), static_cast<int>(msg->pingRate),
        static_cast<int>(msg->networkSpeed),
        static_cast<int>(msg->gammaCorrection),
        static_cast<int>(msg->flags), msg->range, msg->gainPercent,
        msg->speedOfSound, msg->salinity);
  } else if (!forceV1 && raw.size() >= sizeof(OculusSimpleFireMessage2)) {
    auto msg = reinterpret_cast<const OculusSimpleFireMessage2 *>(raw.data());
    spdlog::info(
        "FireMessage v2 fields: mode={} pingRate={} netSpeed={} gamma={} "
        "flags=0x{:02x} rangePct={} gain={} sos={} salinity={} ext=0x{:08x}",
        static_cast<int>(msg->masterMode), static_cast<int>(msg->pingRate),
        static_cast<int>(msg->networkSpeed),
        static_cast<int>(msg->gammaCorrection),
        static_cast<int>(msg->flags), msg->rangePercent, msg->gainPercent,
        msg->speedOfSound, msg->salinity, msg->extFlags);
  }
}

void register_on_connect(DataRx &data_rx, const SonarConfiguration &config,
                         const AppOptions &opt) {
  data_rx.setOnConnectCallback([&]() {
    if (!opt.statsOnly) {
      std::vector<std::string> dump_vec;
      config.dump(dump_vec);

      for (auto const &l : dump_vec) {
        spdlog::debug("Config: {}", l);
      }
    }

    if (opt.fireDump) {
      dump_fire_message(config, opt.forceV1);
    }

    if (opt.forceV1) {
      spdlog::info("Sending FireMessage v1");
      data_rx.sendSimpleFireMessage<OculusSimpleFireMessage>(config);
    } else {
      spdlog::info("Sending FireMessage v2");
      data_rx.sendSimpleFireMessage(config);
    }
  });
}

void register_status_callback(StatusRx &status_rx, DataRx &data_rx,
                              const AppOptions &opt) {
  status_rx.setCallback([&](const SonarStatus &status, bool is_valid) {
    if (opt.printStatus) {
      std::vector<std::string> dump_vec;
      status.dump(dump_vec);
      for (auto const &l : dump_vec) {
        spdlog::info("Status: {}", l);
      }
    }

    if (opt.ipAddr != "auto") {
      return;
    }

    if (!is_valid || data_rx.isConnected()) {
      return;
    }

    data_rx.connect(status.ipAddr());
  });
}

void register_data_callbacks(DataRx &data_rx, const SonarConfiguration &config,
                             const AppOptions &opt, RuntimeContext &ctx) {
  // Callback for a SimplePingResultV1
  data_rx.setCallback<liboculus::SimplePingResultV1>(
      [&](const liboculus::SimplePingResultV1 &ping) {
        if (!opt.statsOnly) {
          spdlog::debug("PingV1: begin");
        }

        {
          const auto valid = checkPingAgreesWithConfig(ping, config);
          if (!valid) {
            spdlog::warn("Mismatch between requested config and ping");
          }
        }

        if (!opt.statsOnly) {
          std::vector<std::string> dump_vec;
          spdlog::debug("PingV1: dump");
          ping.dump(dump_vec);

          for (auto const &l : dump_vec) {
            spdlog::debug("PingV1: {}", l);
          }
        }

        if (ctx.output && ctx.output->is_open()) {
          const char *cdata =
              reinterpret_cast<const char *>(ping.buffer()->data());
          ctx.output->write(cdata, ping.buffer()->size());
        }

        if (ctx.computeMean) {
          spdlog::debug("PingV1: mean");
          spdlog::info("Average intensity: {}",
                       mean_image_intensity(ping.image()));
        }

        if (opt.compareFire) {
          spdlog::info(
              "PingV1 vs Fire: range={}m gain={} fireFlags=0x{:02x} dataSize={} beams={} simpleReturn={}",
              ping.fireMsg()->range, ping.fireMsg()->gainPercent,
              static_cast<int>(ping.fireMsg()->flags),
              static_cast<int>(ping.ping()->dataSize), ping.ping()->nBeams,
              ping.flags().getSimpleReturn() ? "true" : "false");
        }

        write_fields_v1(ctx, ping);
        write_png_frame(opt, ctx, ping.image());

        handle_frame_stop(opt, ctx.count);
        if (!opt.statsOnly) {
          spdlog::debug("PingV1: end");
        }
      });

  // Callback for a SimplePingResultV2
  data_rx.setCallback<liboculus::SimplePingResultV2>(
      [&](const liboculus::SimplePingResultV2 &ping) {
        if (!opt.statsOnly) {
          spdlog::debug("PingV2: begin");
        }

        {
          const auto valid = checkPingAgreesWithConfig(ping, config);
          if (!valid) {
            spdlog::warn("Mismatch between requested config and ping");
          }
        }

        if (!opt.statsOnly) {
          std::vector<std::string> dump_vec;
          spdlog::debug("PingV2: dump");
          ping.dump(dump_vec);

          for (auto const &l : dump_vec) {
            spdlog::debug("PingV2: {}", l);
          }
        }

        if (ctx.output && ctx.output->is_open()) {
          const char *cdata =
              reinterpret_cast<const char *>(ping.buffer()->data());
          ctx.output->write(cdata, ping.buffer()->size());
        }

        if (ctx.computeMean) {
          spdlog::debug("PingV2: mean");
          spdlog::debug("Average intensity: {}",
                        mean_image_intensity(ping.image()));
        }

        if (opt.compareFire) {
          spdlog::info(
              "PingV2 vs Fire: range={}m gain={} fireFlags=0x{:02x} dataSize={} beams={} simpleReturn={}",
              ping.fireMsg()->rangePercent, ping.fireMsg()->gainPercent,
              static_cast<int>(ping.fireMsg()->flags),
              static_cast<int>(ping.ping()->dataSize), ping.ping()->nBeams,
              ping.flags().getSimpleReturn() ? "true" : "false");
        }

        write_fields_v2(ctx, ping);
        write_png_frame(opt, ctx, ping.image());

        handle_frame_stop(opt, ctx.count);
        if (!opt.statsOnly) {
          spdlog::debug("PingV2: end");
        }
      });
}

template <typename PingT>
void register_playback_callback(SonarPlayerBase &player, const char *version,
                                std::ofstream &output, int &count) {
  player.setCallback<PingT>([&](const PingT &ping) {
    spdlog::debug("Playback {}: begin", version);
    // Pings are only sent to the callback if valid()
    // don't need to check independently

    std::vector<std::string> dump_vec;
    try {
      ping.dump(dump_vec);
    } catch (const std::exception &e) {
      spdlog::error("Playback {}: dump exception: {}", version, e.what());
      return;
    } catch (...) {
      spdlog::error("Playback {}: dump exception: unknown", version);
      return;
    }

    for (auto const &l : dump_vec) {
      spdlog::debug("Ping{}: {}", version, l);
    }

    if (output.is_open()) {
      const char *cdata =
          reinterpret_cast<const char *>(ping.buffer()->data());
      output.write(cdata, ping.buffer()->size());
      spdlog::debug("Playback {}: wrote {} bytes", version,
                    ping.buffer()->size());
    }

    try {
      spdlog::info("Average intensity: {}",
                   mean_image_intensity(ping.image()));
    } catch (const std::exception &e) {
      spdlog::error("Playback {}: mean exception: {}", version, e.what());
      return;
    } catch (...) {
      spdlog::error("Playback {}: mean exception: unknown", version);
      return;
    }

    count++;
    spdlog::debug("Playback {}: end", version);
  });
}

void configure_cli(CLI::App &app, AppOptions &opt) {
  app.add_flag("-v,--verbose", opt.verbosity,
               "Additional output (use -vv for even more!)");

  app.add_option("--ip", opt.ipAddr,
                 "IP address of sonar or \"auto\" to automatically detect.");
  app.allow_extras();

  app.add_option("-o,--output", opt.outputFilename,
                 "Saves raw sonar data to specified file.");

  // Playback currently not working
  app.add_option("-i,--input", opt.inputFilename,
                 "Reads raw sonar data from specified file.   Plays file "
                 "contents rather than contacting \"real\" sonar on network.");

  app.add_option("-b,--bits", opt.bitDepth, "Bit depth oof data (8,16,32)");

  app.add_option("-n,--frames", opt.stopAfter, "Stop after (n) frames.");

  app.add_option("-r,--range", opt.range, "Range in meters");

  app.add_option("-g, --gain", opt.gain, "Gain as a percentage (1-100)");

  app.add_option("--beams", opt.beams, "Number of beams (256 or 512)");

  app.add_flag("--no-gain", opt.noGain, "Do not include gain data in returns");

  app.add_flag("--full-return", opt.fullReturn,
               "Request full return messages");

  app.add_flag("--fire-v1", opt.forceV1, "Send OculusSimpleFireMessage v1");

  app.add_flag("--print-status", opt.printStatus,
               "Print key fields from status packets");

  app.add_flag("--stats", opt.statsOnly, "Only print FPS stats");

  app.add_flag("--dump-fire", opt.fireDump,
               "Print FireMessage bytes and key fields");

  app.add_flag("--compare-fire", opt.compareFire,
               "Compare ping fields with last FireMessage");

  app.add_option("--video-dir", opt.videoDir,
                 "Directory to write PNG frames (e.g. out_frames)");

  app.add_option("--video-max", opt.videoMax,
                 "Max value for PNG scaling (0 = auto)");

  app.add_flag("--profile-load", opt.profileLoad,
               "Load settings from occlient.ini");
  app.add_flag("--profile-save", opt.profileSave,
               "Save settings to occlient.ini and exit");

  app.add_option("--dump-fields", opt.fieldsPath,
                 "Write parsed ping fields to a text file");
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

  // Playback callbacks share the same behavior; only labels differ.
  register_playback_callback<liboculus::SimplePingResultV1>(*player, "V1",
                                                            output, count);
  register_playback_callback<liboculus::SimplePingResultV2>(*player, "V2",
                                                            output, count);

  // SimplePingResult ping;
  while (player->nextPing() && !player->eof()) {
    spdlog::debug("Read a ping");
    ;
  }

  spdlog::info("{} sonar packets decoded", count);

  return 0;
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
  AppOptions opt;

  configure_cli(app, opt);

  CLI11_PARSE(app, argc, argv);

  if (opt.ipAddr == "auto") {
    const auto extras = app.remaining();
    if (!extras.empty()) {
      opt.ipAddr = extras.front();
    }
  }

  const std::string profilePath = "occlient.ini";
  if (opt.profileLoad) {
    const auto kv = read_ini_kv(profilePath);
    if (!kv.empty()) {
      apply_profile_kv(kv, opt);
    }
  }

  if (opt.profileSave) {
    return save_profile(opt, profilePath) ? 0 : 1;
  }

  configure_logging(opt);
  if (!validate_options(opt)) {
    return 1;
  }

  ofstream output;
  ofstream fieldsOut;
  std::mutex fieldsMutex;
  if (!open_outputs(opt, output, fieldsOut)) {
    return 1;
  }

  // If playing back an input file, run a different main loop ...
  if (!opt.inputFilename.empty()) {
    spdlog::info("Playing back file {}", opt.inputFilename);
    playbackSonarFile(opt.inputFilename, output, opt.stopAfter);
    return 0;
  }

#ifdef _WIN32
  signal(SIGINT, signalHandler);
#else
  signal(SIGHUP, signalHandler);
#endif

  spdlog::debug("Starting loop");

  // Build the sonar configuration from CLI/profile settings.
  const SonarConfiguration config = build_config(opt);

  _io_thread.reset(new IoServiceThread);
  DataRx _data_rx(_io_thread->context());
  StatusRx _status_rx(_io_thread->context());

  RuntimeContext ctx;
  ctx.output = &output;
  ctx.fieldsOut = &fieldsOut;
  ctx.fieldsMutex = &fieldsMutex;
  ctx.computeMean = (!opt.statsOnly && opt.verbosity > 0);

  register_data_callbacks(_data_rx, config, opt, ctx);
  register_on_connect(_data_rx, config, opt);
  register_status_callback(_status_rx, _data_rx, opt);

  if (opt.ipAddr != "auto") {
    // Otherwise, just (attempt to) connect the DataRx to the specified IP
    // address
    _data_rx.connect(opt.ipAddr);
  }
  _io_thread->start();

  int lastCount = 0;
  while (!doStop.load(std::memory_order_relaxed)) {
    // Very rough Hz calculation right now
    const auto c = ctx.count.load(std::memory_order_relaxed);
    if (opt.statsOnly) {
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
