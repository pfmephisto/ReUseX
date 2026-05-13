// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "database/input_handler.hpp"

#include <algorithm>
#include <iostream>
#include <spdlog/spdlog.h>
#include <sstream>

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

#ifdef __unix__
#include <unistd.h>
#endif

namespace rux::database {

bool InputHandler::is_stdin_available() {
#ifdef __unix__
  return isatty(STDIN_FILENO) == 0; // stdin is NOT a TTY (redirected/piped)
#elif defined(_WIN32)
  return _isatty(_fileno(stdin)) == 0;
#else
  return false; // Conservative default
#endif
}

InputSource
InputHandler::detect_source(const std::optional<std::string> &inline_value) {
  if (is_stdin_available()) {
    return InputSource::Stdin;
  } else if (inline_value.has_value() && !inline_value->empty()) {
    return InputSource::Inline;
  } else {
    return InputSource::None;
  }
}

std::vector<uint8_t> InputHandler::read_binary_from_stdin() {
#ifdef _WIN32
  // Set stdin to binary mode on Windows
  _setmode(_fileno(stdin), _O_BINARY);
#endif

  std::vector<uint8_t> data;
  constexpr size_t kBufferSize = 8192;
  char buffer[kBufferSize];

  while (std::cin.read(buffer, kBufferSize) || std::cin.gcount() > 0) {
    data.insert(data.end(), buffer, buffer + std::cin.gcount());
  }

  spdlog::trace("Read {} bytes from stdin", data.size());
  return data;
}

std::string InputHandler::read_text_from_stdin() {
  std::ostringstream ss;
  ss << std::cin.rdbuf();
  std::string text = ss.str();

  spdlog::trace("Read {} bytes of text from stdin", text.size());
  return text;
}

DataPayload
InputHandler::read_input(const std::optional<std::string> &inline_value) {
  auto source = detect_source(inline_value);

  switch (source) {
  case InputSource::Stdin: {
    spdlog::debug("Reading input from stdin");
    auto data = read_binary_from_stdin();

    // Heuristic: treat as binary if it contains any NUL byte, otherwise text.
    // NUL is forbidden in UTF-8 / ASCII / common text formats (incl. JSON)
    // and is reliably present in nearly every binary container (PNG, JPEG,
    // PLY, OBJ binary, HDF5, ...). False positives are rare and the caller
    // can always force text by using an inline value.
    const bool is_binary =
        std::find(data.begin(), data.end(), '\0') != data.end();

    if (is_binary) {
      spdlog::debug("Stdin contains NUL bytes — treating as binary");
      return data;
    }

    spdlog::debug("Stdin has no NUL bytes — treating as text");
    return std::string(data.begin(), data.end());
  }

  case InputSource::Inline: {
    spdlog::debug("Using inline value as input");
    return *inline_value;
  }

  case InputSource::None:
  default:
    throw std::runtime_error(
        "No input provided. Use shell redirection (< file), pipe (cat file |), "
        "or provide an inline value.");
  }
}

} // namespace rux::database
