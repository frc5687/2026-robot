// Team 5687 2026

#pragma once

#include <frc/Timer.h>
#include <units/current.h>
#include <units/energy.h>
#include <units/power.h>
#include <units/voltage.h>

#include <functional>
#include <string>
#include <vector>

class SubsystemBatteryLogger {
public:
  using CurrentProvider = std::function<units::ampere_t()>;
  using PowerProvider = std::function<units::watt_t()>;

  void RegisterSubsystem(std::string name, CurrentProvider currentProvider,
                         PowerProvider powerProvider);

  void RegisterAggregateGroup(std::string name, CurrentProvider currentProvider,
                              PowerProvider powerProvider);

  void Update();

private:
  struct TrackedSubsystem {
    std::string name;
    CurrentProvider currentProvider;
    PowerProvider powerProvider;
    units::joule_t accumulatedEnergy{0.0};
  };

  std::vector<TrackedSubsystem> m_subsystems;
  std::vector<TrackedSubsystem> m_aggregateGroups;
  units::joule_t m_totalEnergy{0.0};
  units::second_t m_lastTimestamp{0_s};
  bool m_hasLastTimestamp{false};
};
