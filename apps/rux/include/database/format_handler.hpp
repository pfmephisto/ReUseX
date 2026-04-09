// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <variant>
#include <vector>

namespace rux::database {

/**
 * @brief Output format types
 */
enum class OutputFormat {
  Binary, // Raw binary data (point clouds, meshes, images)
  JSON,   // JSON object or array
  Text    // Plain text (scalar values)
};

/**
 * @brief Data payload that can be returned by routers
 *
 * Variant of:
 * - std::string (text data)
 * - std::vector<uint8_t> (binary data)
 * - nlohmann::json (structured data)
 */
using DataPayload =
    std::variant<std::string, std::vector<uint8_t>, nlohmann::json>;

/**
 * @brief Detect format from payload type
 *
 * @param payload Data payload
 * @return Detected output format
 */
OutputFormat detect_format(const DataPayload &payload);

/**
 * @brief Write data to stdout with appropriate formatting
 *
 * Handles:
 * - Binary data (sets stdout to binary mode on Windows)
 * - JSON data (pretty-printed for TTY, compact for pipes)
 * - Text data (plain output)
 *
 * @param payload Data to write
 * @param force_pretty Force pretty-print JSON even for non-TTY
 */
void write_output(const DataPayload &payload, bool force_pretty = false);

/**
 * @brief Write data to a file
 *
 * @param payload Data to write
 * @param file_path Output file path
 */
void write_to_file(const DataPayload &payload,
                   const std::filesystem::path &file_path);

/**
 * @brief Check if stdout is a TTY (for format detection)
 *
 * @return true if stdout is connected to a terminal
 */
bool is_stdout_tty();

/**
 * @brief Convert JSON to text (for scalar values)
 *
 * Extracts the value from JSON and returns as plain text string.
 * Works for strings, numbers, booleans.
 *
 * @param j JSON value
 * @return Text representation
 */
std::string json_to_text(const nlohmann::json &j);

} // namespace rux::database
