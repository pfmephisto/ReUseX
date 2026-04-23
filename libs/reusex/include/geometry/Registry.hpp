// SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
//
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include "reusex/core/logging.hpp"

#include <boost/property_map/property_map.hpp>

#include <map>
#include <memory>
#include <string>
#include <typeindex>
#include <utility>

namespace reusex::geometry {
class Registry {
    private:
  // Keyed by name and type_index
  std::map<std::pair<std::string, std::type_index>, std::shared_ptr<void>>
      registry;

    public:
  virtual ~Registry() = default;
  template <typename Key, typename T>
  std::pair<boost::associative_property_map<std::map<Key, T>>, bool>
  add_property_map(const std::string &name) {
    auto key = std::make_pair(name, std::type_index(typeid(T)));
    auto it = registry.find(key);
    if (it != registry.end()) {
      // Already exists: recover the map and wrap it in a property_map
      auto map_ptr = std::static_pointer_cast<std::map<Key, T>>(it->second);
      return {boost::associative_property_map<std::map<Key, T>>(*map_ptr),
              false};
    } else {
      // Create new map and store in registry
      auto m = std::make_shared<std::map<Key, T>>();
      registry[key] = m;
      return {boost::associative_property_map<std::map<Key, T>>(*m), true};
    }
  }

  template <typename Key, typename T>
  const boost::associative_property_map<std::map<Key, T>>
  property_map(const std::string &name) const {
    auto key = std::make_pair(name, std::type_index(typeid(T)));
    auto it = registry.find(key);
    if (it == registry.end()) {
      reusex::error("Property map not found: {}", name);
      throw std::runtime_error("Property map not found");
    }
    auto map_ptr = std::static_pointer_cast<std::map<Key, T>>(it->second);
    return boost::associative_property_map<std::map<Key, T>>(*map_ptr);
  }
};
} // namespace reusex::geometry
