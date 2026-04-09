// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace rux::database {

/**
 * @brief Exception thrown when path parsing fails
 */
class PathError : public std::runtime_error {
public:
  explicit PathError(const std::string &message)
      : std::runtime_error(message) {}
};

/**
 * @brief Type of path component
 */
enum class ComponentType {
  Collection, // e.g., "clouds", "meshes", "frames"
  Item,       // e.g., "mycloud", "123"
  Property,   // e.g., "metadata", "point_count"
  Index       // e.g., "[0]", "[5]"
};

/**
 * @brief Represents a single component in a resource path
 *
 * Examples:
 * - clouds → {Collection, "clouds"}
 * - mycloud → {Item, "mycloud"}
 * - metadata → {Property, "metadata"}
 * - [0] → {Index, "", 0}
 */
struct PathComponent {
  ComponentType type;
  std::string value;
  std::optional<int> index;

  PathComponent(ComponentType t, std::string v)
      : type(t), value(std::move(v)), index(std::nullopt) {}

  PathComponent(ComponentType t, std::string v, int idx)
      : type(t), value(std::move(v)), index(idx) {}

  bool is_collection() const { return type == ComponentType::Collection; }
  bool is_item() const { return type == ComponentType::Item; }
  bool is_property() const { return type == ComponentType::Property; }
  bool is_index() const { return type == ComponentType::Index; }
  bool has_wildcard() const { return value.find('*') != std::string::npos; }
};

/**
 * @brief Parse a resource path into components
 *
 * Syntax:
 * - Dot notation (primary): "clouds.mycloud.metadata"
 * - Slash notation (backward compat): "clouds/mycloud/metadata"
 * - Array indexing: "clouds[0]" or "clouds[0].metadata"
 * - Wildcards: "clouds.*" or "clouds.scan*"
 *
 * Grammar:
 * path ::= collection
 *        | collection.item
 *        | collection.item.property
 *        | collection[index]
 *        | collection[index].property
 *
 * @param path Resource path to parse
 * @return Vector of path components
 * @throws PathError if path is invalid
 *
 * Examples:
 * - "clouds" → [{Collection, "clouds"}]
 * - "clouds.scan1" → [{Collection, "clouds"}, {Item, "scan1"}]
 * - "clouds.scan1.metadata" → [{Collection, "clouds"}, {Item, "scan1"},
 *                               {Property, "metadata"}]
 * - "clouds[0]" → [{Collection, "clouds"}, {Index, "", 0}]
 * - "clouds.*" → [{Collection, "clouds"}, {Item, "*"}]
 */
std::vector<PathComponent> parse_path(std::string_view path);

/**
 * @brief Check if a collection name is valid
 *
 * Valid collections: clouds, materials, meshes, projects
 *
 * @param name Collection name to validate
 * @return true if valid collection name
 */
bool is_valid_collection(std::string_view name);

/**
 * @brief Expand wildcard patterns in item names
 *
 * @param pattern Pattern with wildcards (e.g., "scan*", "*_preprocessed")
 * @param items Available items to match against
 * @return Vector of matching item names
 */
std::vector<std::string>
expand_wildcard(std::string_view pattern,
                const std::vector<std::string> &items);

/**
 * @brief Check if a string matches a wildcard pattern
 *
 * @param str String to match
 * @param pattern Pattern with wildcards
 * @return true if string matches pattern
 */
bool matches_wildcard(std::string_view str, std::string_view pattern);

} // namespace rux::database
