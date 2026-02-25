// Team 5687 2026

#pragma once

#include <networktables/BooleanArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/FloatArrayTopic.h>
#include <networktables/FloatTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <wpi/struct/Struct.h>

#include <shared_mutex>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>


struct StringHash {
  using is_transparent = void;
  size_t operator()(std::string_view sv) const noexcept {
    return std::hash<std::string_view>{}(sv);
  }
};

class Logger {
public:
  static Logger &Instance() {
    static Logger inst;
    return inst;
  }

  template <typename T>
  void Log(std::string_view name, const T &value, int64_t ts = 0) {
    if constexpr (std::is_same_v<T, bool>) {
      auto &pub = GetOrCreate(name, m_boolPubs, [&] {
        return m_instance.GetBooleanTopic(name).Publish();
      });
      pub.Set(value, ts);

    } else if constexpr (std::is_same_v<T, int>) {
      auto &pub = GetOrCreate(name, m_intPubs, [&] {
        return m_instance.GetIntegerTopic(name).Publish();
      });
      pub.Set(static_cast<int64_t>(value), ts);

    } else if constexpr (std::is_same_v<T, float>) {
      auto &pub = GetOrCreate(name, m_floatPubs, [&] {
        return m_instance.GetFloatTopic(name).Publish();
      });
      pub.Set(value, ts);

    } else if constexpr (std::is_same_v<T, double>) {
      auto &pub = GetOrCreate(name, m_doublePubs, [&] {
        return m_instance.GetDoubleTopic(name).Publish();
      });
      pub.Set(value, ts);

    } else if constexpr (std::is_same_v<T, std::string>) {
      auto &pub = GetOrCreate(name, m_stringPubs, [&] {
        return m_instance.GetStringTopic(name).Publish();
      });
      pub.Set(value, ts);

    } else if constexpr (wpi::StructSerializable<T>) {
      auto &pub = GetOrCreate(name, GetStructMap<T>(), [&] {
        return m_instance.GetStructTopic<T>(name).Publish();
      });
      pub.Set(value, ts);

    } else {
      static_assert(!sizeof(T), "Type not supported");
    }
  }

  template <typename T>
  void Log(std::string_view name, const std::vector<T> &values,
           int64_t ts = 0) {
    if constexpr (std::is_same_v<T, bool>) {
      std::vector<int> tmp;
      tmp.reserve(values.size());
      for (bool b : values)
        tmp.push_back(b ? 1 : 0);
      auto &pub = GetOrCreate(name, m_boolArrPubs, [&] {
        return m_instance.GetBooleanArrayTopic(name).Publish();
      });
      pub.Set(tmp, ts);

    } else if constexpr (std::is_same_v<T, int>) {
      std::vector<int64_t> tmp(values.begin(), values.end());
      auto &pub = GetOrCreate(name, m_intArrPubs, [&] {
        return m_instance.GetIntegerArrayTopic(name).Publish();
      });
      pub.Set(tmp, ts);

    } else if constexpr (std::is_same_v<T, float>) {
      auto &pub = GetOrCreate(name, m_floatArrPubs, [&] {
        return m_instance.GetFloatArrayTopic(name).Publish();
      });
      pub.Set(values, ts);

    } else if constexpr (std::is_same_v<T, double>) {
      auto &pub = GetOrCreate(name, m_doubleArrPubs, [&] {
        return m_instance.GetDoubleArrayTopic(name).Publish();
      });
      pub.Set(values, ts);

    } else if constexpr (std::is_same_v<T, std::string>) {
      auto &pub = GetOrCreate(name, m_stringArrPubs, [&] {
        return m_instance.GetStringArrayTopic(name).Publish();
      });
      pub.Set(values, ts);

    } else if constexpr (wpi::StructSerializable<T>) {
      auto &pub = GetOrCreate(name, GetStructArrMap<T>(), [&] {
        return m_instance.GetStructArrayTopic<T>(name).Publish();
      });
      pub.Set(values, ts);

    } else {
      static_assert(!sizeof(T), "Type not supported");
    }
  }

  template <typename T, size_t N>
  void Log(std::string_view name, const std::array<T, N> &arr,
           int64_t ts = 0) {
    if constexpr (wpi::StructSerializable<T>) {
      auto &pub = GetOrCreate(name, GetStructArrMap<T>(), [&] {
        return m_instance.GetStructArrayTopic<T>(name).Publish();
      });
      pub.Set(arr, ts);
    } else if constexpr (std::is_same_v<T, double>) {
      auto &pub = GetOrCreate(name, m_doubleArrPubs, [&] {
        return m_instance.GetDoubleArrayTopic(name).Publish();
      });
      pub.Set(std::span<const double>{arr.data(), N}, ts);
    } else if constexpr (std::is_same_v<T, float>) {
      auto &pub = GetOrCreate(name, m_floatArrPubs, [&] {
        return m_instance.GetFloatArrayTopic(name).Publish();
      });
      pub.Set(std::span<const float>{arr.data(), N}, ts);
    } else {
      // Fallback for types needing conversion (bool→int, int→int64_t)
      std::vector<T> tmp(arr.begin(), arr.end());
      Log(name, tmp, ts);
    }
  }

private:
  Logger() : m_instance(nt::NetworkTableInstance::GetDefault()) {}

  // Hot path: shared (read) lock finds existing publisher — no allocation.
  // Cold path: exclusive (write) lock creates publisher once per key.
  template <class Map, class Factory>
  typename Map::mapped_type &GetOrCreate(std::string_view name, Map &map,
                                         Factory &&make) {
    {
      std::shared_lock rl(m_lock);
      if (auto it = map.find(name); it != map.end())
        return it->second;
    }
    std::unique_lock wl(m_lock);
    if (auto it = map.find(name); it != map.end())
      return it->second;
    auto [it, _] =
        map.emplace(std::string{name}, std::forward<Factory>(make)());
    return it->second;
  }

  template <typename T> static auto &GetStructMap() {
    static std::unordered_map<std::string, nt::StructPublisher<T>, StringHash,
                              std::equal_to<>>
        m;
    return m;
  }
  template <typename T> static auto &GetStructArrMap() {
    static std::unordered_map<std::string, nt::StructArrayPublisher<T>,
                              StringHash, std::equal_to<>>
        m;
    return m;
  }

  nt::NetworkTableInstance m_instance;
  std::shared_mutex m_lock;

  using SMap = std::equal_to<>;
  std::unordered_map<std::string, nt::BooleanPublisher, StringHash, SMap>
      m_boolPubs;
  std::unordered_map<std::string, nt::IntegerPublisher, StringHash, SMap>
      m_intPubs;
  std::unordered_map<std::string, nt::FloatPublisher, StringHash, SMap>
      m_floatPubs;
  std::unordered_map<std::string, nt::DoublePublisher, StringHash, SMap>
      m_doublePubs;
  std::unordered_map<std::string, nt::StringPublisher, StringHash, SMap>
      m_stringPubs;

  std::unordered_map<std::string, nt::BooleanArrayPublisher, StringHash, SMap>
      m_boolArrPubs;
  std::unordered_map<std::string, nt::IntegerArrayPublisher, StringHash, SMap>
      m_intArrPubs;
  std::unordered_map<std::string, nt::FloatArrayPublisher, StringHash, SMap>
      m_floatArrPubs;
  std::unordered_map<std::string, nt::DoubleArrayPublisher, StringHash, SMap>
      m_doubleArrPubs;
  std::unordered_map<std::string, nt::StringArrayPublisher, StringHash, SMap>
      m_stringArrPubs;
};
