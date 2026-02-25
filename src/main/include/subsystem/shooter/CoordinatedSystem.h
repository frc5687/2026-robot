// Team 5687 2026

#pragma once

// This countians a way to coordinate subsystems into one group that runs after
// all subsystem
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/time.h>

#include "utils/Logger.h"

class CoordinatedSystem {
public:
  explicit CoordinatedSystem(const std::string &name);
  virtual void Periodic() final;
  virtual void Update() {}

protected:
  virtual void LogTelemetry() {};

  std::shared_ptr<nt::NetworkTable> GetTable() const { return m_table; }
  template <typename T> void Log(const std::string &key, const T &value) {
    Logger::Instance().Log(m_name + "/" + key, value);
  }

private:
  std::string m_name;
  std::shared_ptr<nt::NetworkTable> m_table;
  units::second_t m_lastLogTime{0};

  units::second_t m_totalUpdateTime{0};
  size_t m_updateCount{0};
};
