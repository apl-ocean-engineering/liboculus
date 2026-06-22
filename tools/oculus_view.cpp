#include <algorithm>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "liboculus/thirdparty/CLI11/CLI11.hpp"

#include "spdlog/spdlog.h"

#include "liboculus/Logger.h"
#include "liboculus/SonarPlayer.h"

namespace {

struct State {
  int target_index = 0;
  int seen = 0;
  bool done = false;
  bool success = false;
  uint32_t max_override = 0;
  std::string output_path;
  std::string error;
};

uint32_t compute_max(const liboculus::ImageData &image) {
  uint32_t max_val = 0;
  const uint32_t width = static_cast<uint32_t>(image.nBeams());
  const uint32_t height = static_cast<uint32_t>(image.nRanges());
  for (uint32_t r = 0; r < height; ++r) {
    for (uint32_t b = 0; b < width; ++b) {
      max_val = std::max(max_val, image.at_uint32(b, r));
    }
  }
  return max_val;
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
    max_val = compute_max(image);
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

  // PNG signature
  const uint8_t signature[8] = {0x89, 'P', 'N', 'G',
                                0x0D, 0x0A, 0x1A, 0x0A};
  out.write(reinterpret_cast<const char *>(signature), 8);

  // IHDR
  std::vector<uint8_t> ihdr(13, 0);
  ihdr[0] = static_cast<uint8_t>((width >> 24) & 0xFF);
  ihdr[1] = static_cast<uint8_t>((width >> 16) & 0xFF);
  ihdr[2] = static_cast<uint8_t>((width >> 8) & 0xFF);
  ihdr[3] = static_cast<uint8_t>(width & 0xFF);
  ihdr[4] = static_cast<uint8_t>((height >> 24) & 0xFF);
  ihdr[5] = static_cast<uint8_t>((height >> 16) & 0xFF);
  ihdr[6] = static_cast<uint8_t>((height >> 8) & 0xFF);
  ihdr[7] = static_cast<uint8_t>(height & 0xFF);
  ihdr[8] = 8;  // bit depth
  ihdr[9] = 0;  // color type: grayscale
  ihdr[10] = 0; // compression
  ihdr[11] = 0; // filter
  ihdr[12] = 0; // interlace

  if (!write_chunk(out, "IHDR", ihdr, error)) {
    return false;
  }

  // Prepare raw scanlines with filter byte (0) per row
  std::vector<uint8_t> raw;
  raw.reserve(height * (width + 1));
  for (uint32_t r = 0; r < height; ++r) {
    raw.push_back(0);
    const size_t row_offset = static_cast<size_t>(r) * width;
    raw.insert(raw.end(), pixels.begin() + row_offset,
               pixels.begin() + row_offset + width);
  }

  // Zlib stream with uncompressed DEFLATE blocks
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

template <typename PingT>
void handle_ping(const PingT &ping, State &state) {
  if (state.done) {
    return;
  }
  if (state.seen == state.target_index) {
    spdlog::info("Selected ping {} (beams={}, ranges={}, dataSize={})",
                 state.target_index, ping.ping()->nBeams,
                 ping.ping()->nRanges,
                 static_cast<int>(ping.ping()->dataSize));
    if (!write_png(ping.image(), state.max_override, state.output_path,
                   state.error)) {
      state.success = false;
    } else {
      state.success = true;
    }
    state.done = true;
  }
  state.seen++;
}

} // namespace

int main(int argc, char **argv) {
  auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  liboculus::Logger::add_sink(stdout_sink);
  spdlog::set_default_logger(
      std::make_shared<spdlog::logger>("ocview", stdout_sink));

  CLI::App app{"Simple Oculus sonar image dumper"};

  int verbosity = 0;
  app.add_flag("-v,--verbose", verbosity,
               "Additional output (use -vv for even more!)");

  std::string input;
  std::string output = "out.png";
  int ping_index = 0;
  uint32_t max_override = 0;

  app.add_option("-i,--input", input,
                 "Raw sonar data file (e.g. *.raw).")->required();
  app.add_option("-o,--output", output, "Output PNG file.");
  app.add_option("--ping", ping_index, "0-based ping index to export.")
      ->default_val(0);
  app.add_option("--max", max_override,
                 "Max value for scaling (0 = auto).")
      ->default_val(0);

  CLI11_PARSE(app, argc, argv);

  if (verbosity == 1) {
    spdlog::set_level(spdlog::level::debug);
  } else if (verbosity > 1) {
    spdlog::set_level(spdlog::level::trace);
  }

  if (ping_index < 0) {
    spdlog::error("Ping index must be >= 0");
    return 1;
  }

  auto player = liboculus::SonarPlayerBase::OpenFile(input);
  if (!player) {
    spdlog::error("Unable to open sonar file {}", input);
    return 1;
  }
  if (!player->open(input)) {
    spdlog::error("Failed to open {}", input);
    return 1;
  }

  State state;
  state.target_index = ping_index;
  state.max_override = max_override;
  state.output_path = output;

  player->setCallback<liboculus::SimplePingResultV1>(
      [&](const liboculus::SimplePingResultV1 &ping) {
        handle_ping(ping, state);
      });
  player->setCallback<liboculus::SimplePingResultV2>(
      [&](const liboculus::SimplePingResultV2 &ping) {
        handle_ping(ping, state);
      });

  while (!state.done && player->nextPing() && !player->eof()) {
    ;
  }

  if (!state.done) {
    spdlog::error("Ping index {} out of range (decoded {}).", ping_index,
                  state.seen);
    return 1;
  }
  if (!state.success) {
    spdlog::error("Failed to write PGM: {}", state.error);
    return 1;
  }

  spdlog::info("Wrote {}", output);
  return 0;
}
