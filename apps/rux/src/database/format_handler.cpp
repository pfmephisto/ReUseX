// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/format_handler.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <spdlog/spdlog.h>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#ifdef __unix__
#include <unistd.h>
#endif

namespace rux::database {

OutputFormat detect_format(const DataPayload &payload) {
  if (std::holds_alternative<std::vector<uint8_t>>(payload)) {
    return OutputFormat::Binary;
  } else if (std::holds_alternative<nlohmann::json>(payload)) {
    return OutputFormat::JSON;
  } else {
    return OutputFormat::Text;
  }
}

bool is_stdout_tty() {
#ifdef __unix__
  return isatty(STDOUT_FILENO) == 1;
#elif defined(_WIN32)
  return _isatty(_fileno(stdout)) != 0;
#else
  return false; // Conservative default
#endif
}

std::string json_to_text(const nlohmann::json &j) {
  if (j.is_string()) {
    return j.get<std::string>();
  } else if (j.is_number()) {
    return std::to_string(j.get<double>());
  } else if (j.is_boolean()) {
    return j.get<bool>() ? "true" : "false";
  } else if (j.is_null()) {
    return "null";
  } else {
    // For complex types, return JSON string
    return j.dump();
  }
}

void write_output(const DataPayload &payload, bool force_pretty) {
  auto format = detect_format(payload);

  if (format == OutputFormat::Binary) {
    const auto &data = std::get<std::vector<uint8_t>>(payload);

#ifdef _WIN32
    // Set stdout to binary mode on Windows
    _setmode(_fileno(stdout), _O_BINARY);
#endif

    std::cout.write(reinterpret_cast<const char *>(data.data()), data.size());
    std::cout.flush();

    spdlog::trace("Wrote {} bytes of binary data to stdout", data.size());

  } else if (format == OutputFormat::JSON) {
    const auto &json = std::get<nlohmann::json>(payload);

    // Pretty-print for TTY or if forced, compact otherwise
    int indent = (force_pretty || is_stdout_tty()) ? 2 : -1;
    std::cout << json.dump(indent) << std::endl;

    spdlog::trace("Wrote JSON data to stdout (pretty={})",
                  indent > 0 ? "true" : "false");

  } else { // Text
    const auto &text = std::get<std::string>(payload);
    std::cout << text << std::endl;

    spdlog::trace("Wrote {} bytes of text data to stdout", text.size());
  }
}

void write_to_file(const DataPayload &payload,
                   const std::filesystem::path &file_path) {
  auto format = detect_format(payload);

  if (format == OutputFormat::Binary) {
    const auto &data = std::get<std::vector<uint8_t>>(payload);

    std::ofstream file(file_path, std::ios::binary);
    if (!file) {
      throw std::runtime_error("Failed to open file for writing: " +
                               file_path.string());
    }

    file.write(reinterpret_cast<const char *>(data.data()), data.size());
    file.close();

    spdlog::info("Wrote {} bytes to {}", data.size(), file_path.string());

  } else if (format == OutputFormat::JSON) {
    const auto &json = std::get<nlohmann::json>(payload);

    std::ofstream file(file_path);
    if (!file) {
      throw std::runtime_error("Failed to open file for writing: " +
                               file_path.string());
    }

    file << json.dump(2) << std::endl; // Always pretty-print to files
    file.close();

    spdlog::info("Wrote JSON data to {}", file_path.string());

  } else { // Text
    const auto &text = std::get<std::string>(payload);

    std::ofstream file(file_path);
    if (!file) {
      throw std::runtime_error("Failed to open file for writing: " +
                               file_path.string());
    }

    file << text << std::endl;
    file.close();

    spdlog::info("Wrote text data to {}", file_path.string());
  }
}

} // namespace rux::database
