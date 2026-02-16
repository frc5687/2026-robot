// Team 5687 2026

#pragma once

#include <wpi/interpolating_map.h>

#include <utility>
#include <vector>

template <typename Key, typename Value>
class InterpolatingTreeMap : wpi::interpolating_map<Key, Value> {
 public:
  using Base = wpi::interpolating_map<Key, Value>;
  using Base::Base;

  void InsertValues(const std::vector<std::pair<Key, Value>>& values) {
    for (const auto& entry : values) {
      this->insert(entry.first, entry.second);
    }
  }

  Value GetValue(const Key& key) const { return this->operator[](key); }
};
