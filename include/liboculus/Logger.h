// Copyright (c) 2017-2026 University of Washington
// Author: Aaron Marburg <amarburg@uw.edu>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of University of Washington nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <memory>
#include <spdlog/common.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <utility>

namespace liboculus {

class Logger {
public:
  static std::shared_ptr<Logger> &
  get_instance(const std::shared_ptr<spdlog::logger> &logger_in = nullptr) {
    static std::shared_ptr<Logger> s_logger(init(logger_in));
    if ((logger_in) && (logger_in != s_logger->logger_))
      s_logger->logger_ = logger_in;
    return s_logger;
  }

  static std::shared_ptr<spdlog::logger> &get_logger() {
    return Logger::get_instance()->logger_;
  }

  static void set_logger(const std::shared_ptr<spdlog::logger> &s) {
    Logger::get_instance(s);
  }

  static void add_sink(const spdlog::sink_ptr &s) {
    // n.b. per the spdlog documentation, this is _not_ thread safe
    Logger::get_logger()->sinks().push_back(s);
  }

  ~Logger() {}

private:
  static std::shared_ptr<Logger>
  init(const std::shared_ptr<spdlog::logger> &logger_in = nullptr) {
    // Use new to access private constructor
    return std::shared_ptr<Logger>(new Logger(logger_in));
  }

  explicit Logger(const std::shared_ptr<spdlog::logger> &l = nullptr)
      : logger_(l) {
    if (!logger_) {
      logger_ = std::make_shared<spdlog::logger>("liboculus");
      spdlog::register_logger(logger_);
    }
  }

  std::shared_ptr<spdlog::logger> logger_;

  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;
};

// Convenience wrappers around "oclog::"
namespace oclog {
using spdlog::format_string_t;
using spdlog::source_loc;

template <typename... Args>
inline void log(source_loc source, spdlog::level::level_enum lvl,
                format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->log(source, lvl, fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void log(spdlog::level::level_enum lvl, format_string_t<Args...> fmt,
                Args &&...args) {
  Logger::get_logger()->log(source_loc{}, lvl, fmt,
                            std::forward<Args>(args)...);
}

template <typename... Args>
inline void trace(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->trace(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void debug(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->debug(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void info(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->info(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void warn(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->warn(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void error(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->error(fmt, std::forward<Args>(args)...);
}

template <typename... Args>
inline void critical(format_string_t<Args...> fmt, Args &&...args) {
  Logger::get_logger()->critical(fmt, std::forward<Args>(args)...);
}

template <typename T>
inline void log(source_loc source, spdlog::level::level_enum lvl,
                const T &msg) {
  Logger::get_logger()->log(source, lvl, msg);
}

template <typename T>
inline void log(spdlog::level::level_enum lvl, const T &msg) {
  Logger::get_logger()->log(lvl, msg);
}
} // namespace oclog

} // namespace liboculus
