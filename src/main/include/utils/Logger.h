// Team 5687 2026

#pragma once

#include <frc/DataLogManager.h>
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
#include <stdint.h>
#include <wpi/struct/Struct.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

struct StringHash {
  using is_transparent = void;

  size_t operator()(std::string_view sv) const noexcept {
    return std::hash<std::string_view>{}(sv);
  }
};

template <typename T>
concept SupportedInteger = std::is_integral_v<T> && !std::is_same_v<T, bool>;

template <class> inline constexpr bool always_false_v = false;

class Logger {
public:
  static Logger &Instance() {
    static Logger inst;
    return inst;
  }

  template <typename T>
  void Log(std::string_view name, const T &value, int64_t ts = 0) {
    EnqueueLog(name, value, ts);
  }

  template <typename T>
  void Log(std::string_view name, const std::vector<T> &values,
           int64_t ts = 0) {
    EnqueueLog(name, values, ts);
  }

  template <typename T, size_t N>
  void Log(std::string_view name, const std::array<T, N> &arr, int64_t ts = 0) {
    EnqueueLog(name, arr, ts);
  }

  size_t GetDroppedTaskCount() const noexcept {
    return m_droppedTaskCount.load(std::memory_order_relaxed);
  }

  void EnableFileLogger() {
    frc::DataLogManager::Start();
    frc::DataLogManager::LogNetworkTables(true);
    frc::DataLogManager::LogConsoleOutput(true);
  }

private:
  template <typename PublisherT>
  using PublisherMap =
      std::unordered_map<std::string, PublisherT, StringHash, std::equal_to<>>;

  using Task = std::function<void()>;
  static constexpr size_t kMaxQueueSize = 8192;

  Logger() : m_instance(nt::NetworkTableInstance::GetDefault()) {
    m_worker = std::thread([this] { WorkerLoop(); });
  }

  ~Logger() {
    {
      std::lock_guard guard(m_queueMutex);
      m_running = false;
    }
    m_queueCV.notify_all();
    if (m_worker.joinable()) {
      m_worker.join();
    }
  }

  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;

  template <typename Value>
  void EnqueueLog(std::string_view name, Value &&value, int64_t ts) {
    std::string key{name};
    using Stored = std::decay_t<Value>;
    Stored stored = std::forward<Value>(value);

    EnqueueTask([this, key = std::move(key), stored = std::move(stored), ts]() {
      LogSync(key, stored, ts);
    });
  }

  void WorkerLoop() {
    while (true) {
      Task task;
      {
        std::unique_lock lock(m_queueMutex);
        m_queueCV.wait(lock,
                       [this] { return !m_running || !m_taskQueue.empty(); });

        if (!m_running && m_taskQueue.empty()) {
          break;
        }

        task = std::move(m_taskQueue.front());
        m_taskQueue.pop_front();
      }

      task();
    }
  }

  void EnqueueTask(Task task) {
    {
      std::lock_guard guard(m_queueMutex);
      if (m_taskQueue.size() >= kMaxQueueSize) {
        m_droppedTaskCount.fetch_add(1, std::memory_order_relaxed);
        return;
      }
      m_taskQueue.push_back(std::move(task));
    }
    m_queueCV.notify_one();
  }

  template <typename T>
  void LogSync(std::string_view name, const T &value, int64_t ts = 0) {
    if constexpr (std::is_same_v<T, bool>) {
      auto &pub = GetOrCreatePublisher(name, m_boolPubs, [&] {
        return m_instance.GetBooleanTopic(name).Publish();
      });
      pub.Set(value, ts);
    } else if constexpr (SupportedInteger<T>) {
      auto &pub = GetOrCreatePublisher(name, m_intPubs, [&] {
        return m_instance.GetIntegerTopic(name).Publish();
      });
      pub.Set(static_cast<int64_t>(value), ts);
    } else if constexpr (std::is_same_v<T, float>) {
      auto &pub = GetOrCreatePublisher(name, m_floatPubs, [&] {
        return m_instance.GetFloatTopic(name).Publish();
      });
      pub.Set(value, ts);
    } else if constexpr (std::is_same_v<T, double>) {
      auto &pub = GetOrCreatePublisher(name, m_doublePubs, [&] {
        return m_instance.GetDoubleTopic(name).Publish();
      });
      pub.Set(value, ts);
    } else if constexpr (std::is_same_v<T, std::string>) {
      auto &pub = GetOrCreatePublisher(name, m_stringPubs, [&] {
        return m_instance.GetStringTopic(name).Publish();
      });
      pub.Set(value, ts);
    } else if constexpr (wpi::StructSerializable<T>) {
      auto &pub = GetOrCreatePublisher(name, GetStructMap<T>(), [&] {
        return m_instance.GetStructTopic<T>(name).Publish();
      });
      pub.Set(value, ts);
    } else {
      static_assert(always_false_v<T>, "Type not supported");
    }
  }

  template <typename T>
  void LogSync(std::string_view name, const std::vector<T> &values,
               int64_t ts = 0) {
    if constexpr (std::is_same_v<T, bool>) {
      auto &pub = GetOrCreatePublisher(name, m_boolArrPubs, [&] {
        return m_instance.GetBooleanArrayTopic(name).Publish();
      });
      pub.Set(values, ts);
    } else if constexpr (SupportedInteger<T>) {
      std::vector<int64_t> tmp(values.begin(), values.end());
      auto &pub = GetOrCreatePublisher(name, m_intArrPubs, [&] {
        return m_instance.GetIntegerArrayTopic(name).Publish();
      });
      pub.Set(tmp, ts);
    } else if constexpr (std::is_same_v<T, float>) {
      auto &pub = GetOrCreatePublisher(name, m_floatArrPubs, [&] {
        return m_instance.GetFloatArrayTopic(name).Publish();
      });
      pub.Set(values, ts);
    } else if constexpr (std::is_same_v<T, double>) {
      auto &pub = GetOrCreatePublisher(name, m_doubleArrPubs, [&] {
        return m_instance.GetDoubleArrayTopic(name).Publish();
      });
      pub.Set(values, ts);
    } else if constexpr (std::is_same_v<T, std::string>) {
      auto &pub = GetOrCreatePublisher(name, m_stringArrPubs, [&] {
        return m_instance.GetStringArrayTopic(name).Publish();
      });
      pub.Set(values, ts);
    } else if constexpr (wpi::StructSerializable<T>) {
      auto &pub = GetOrCreatePublisher(name, GetStructArrMap<T>(), [&] {
        return m_instance.GetStructArrayTopic<T>(name).Publish();
      });
      pub.Set(values, ts);
    } else {
      static_assert(always_false_v<T>, "Type not supported");
    }
  }

  template <typename T, size_t N>
  void LogSync(std::string_view name, const std::array<T, N> &arr,
               int64_t ts = 0) {
    if constexpr (wpi::StructSerializable<T>) {
      auto &pub = GetOrCreatePublisher(name, GetStructArrMap<T>(), [&] {
        return m_instance.GetStructArrayTopic<T>(name).Publish();
      });
      pub.Set(arr, ts);
    } else if constexpr (std::is_same_v<T, double>) {
      auto &pub = GetOrCreatePublisher(name, m_doubleArrPubs, [&] {
        return m_instance.GetDoubleArrayTopic(name).Publish();
      });
      pub.Set(std::span<const double>{arr.data(), N}, ts);
    } else if constexpr (std::is_same_v<T, float>) {
      auto &pub = GetOrCreatePublisher(name, m_floatArrPubs, [&] {
        return m_instance.GetFloatArrayTopic(name).Publish();
      });
      pub.Set(std::span<const float>{arr.data(), N}, ts);
    } else {
      std::vector<T> tmp(arr.begin(), arr.end());
      LogSync(name, tmp, ts);
    }
  }

  template <class Map, class Factory>
  typename Map::mapped_type &GetOrCreatePublisher(std::string_view name,
                                                  Map &map, Factory &&make) {
    {
      std::shared_lock rl(m_lock);
      if (auto it = map.find(name); it != map.end()) {
        return it->second;
      }
    }

    std::unique_lock wl(m_lock);
    if (auto it = map.find(name); it != map.end()) {
      return it->second;
    }

    auto [it, inserted] =
        map.emplace(std::string{name}, std::forward<Factory>(make)());
    (void)inserted;
    return it->second;
  }

  template <typename T> static auto &GetStructMap() {
    static PublisherMap<nt::StructPublisher<T>> m;
    return m;
  }

  template <typename T> static auto &GetStructArrMap() {
    static PublisherMap<nt::StructArrayPublisher<T>> m;
    return m;
  }

  nt::NetworkTableInstance m_instance;
  std::shared_mutex m_lock;

  bool m_running{true};
  std::thread m_worker;
  std::mutex m_queueMutex;
  std::condition_variable m_queueCV;
  std::deque<Task> m_taskQueue;
  std::atomic<size_t> m_droppedTaskCount{0};

  PublisherMap<nt::BooleanPublisher> m_boolPubs;
  PublisherMap<nt::IntegerPublisher> m_intPubs;
  PublisherMap<nt::FloatPublisher> m_floatPubs;
  PublisherMap<nt::DoublePublisher> m_doublePubs;
  PublisherMap<nt::StringPublisher> m_stringPubs;

  PublisherMap<nt::BooleanArrayPublisher> m_boolArrPubs;
  PublisherMap<nt::IntegerArrayPublisher> m_intArrPubs;
  PublisherMap<nt::FloatArrayPublisher> m_floatArrPubs;
  PublisherMap<nt::DoubleArrayPublisher> m_doubleArrPubs;
  PublisherMap<nt::StringArrayPublisher> m_stringArrPubs;
};
