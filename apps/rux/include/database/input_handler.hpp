// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "format_handler.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace rux::database {

/**
 * @brief Input source for set operations
 */
enum class InputSource {
  Stdin,     // Read from stdin (pipe or redirection)
  Inline,    // Inline value from command line argument
  None       // No input provided (error condition)
};

/**
 * @brief Handles input for set operations
 *
 * Priority:
 * 1. stdin (if redirected or piped, i.e., !isatty(STDIN_FILENO))
 * 2. Inline value (positional argument)
 * 3. Error (no input provided)
 */
class InputHandler {
public:
  /**
   * @brief Detect and read input
   *
   * @param inline_value Optional inline value from command line
   * @return Data payload
   * @throws std::runtime_error if no input is available
   */
  static DataPayload read_input(const std::optional<std::string> &inline_value);

  /**
   * @brief Check if stdin is available (redirected or piped)
   *
   * @return true if stdin is not a TTY (has data to read)
   */
  static bool is_stdin_available();

  /**
   * @brief Read binary data from stdin
   *
   * @return Binary data
   */
  static std::vector<uint8_t> read_binary_from_stdin();

  /**
   * @brief Read text data from stdin
   *
   * @return Text data
   */
  static std::string read_text_from_stdin();

  /**
   * @brief Detect input source
   *
   * @param inline_value Optional inline value from command line
   * @return Input source type
   */
  static InputSource
  detect_source(const std::optional<std::string> &inline_value);
};

} // namespace rux::database
