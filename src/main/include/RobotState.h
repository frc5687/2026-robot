// Team 5687 2026

#pragma once

#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <units/time.h>

#include <string>

#include "subsystem/drive/OdometryData.h"
#include "subsystem/flywheel/FlywheelState.h"
#include "subsystem/turret/TurretState.h"
#include "utils/Logger.h"

// This will store the entire state of subsystems we care about, this includes
// Transforms between systems, positional offsets from base, etc.

// What do I need to do:
// Track pose of all subsystems
// I just pass pose from subsystem or only needed values? angle, velocity, etc?
class RobotState {
 public:
  const units::second_t BUFFER_TIME = 1.5_s;
  static RobotState& Instance() {
    static RobotState inst;
    return inst;
  }
  void AddDriveObservation(const OdometryData& state);
  void AddTurretObservation(const TurretState& state);
  void AddFlywheelObservation(const FlywheelState& state);

  // Interpolated or exterpolated based on time, limit to BUFFER_TIME
  OdometryData GetDriveState(units::second_t timestamp);
  TurretState GetTurretState(units::second_t timestamp);
  FlywheelState GetFlywheelState(units::second_t timestamp);

  void LogState(units::second_t timestamp);

 private:
  RobotState();
  frc::TimeInterpolatableBuffer<OdometryData> m_driveBuffer;
  frc::TimeInterpolatableBuffer<TurretState> m_turretBuffer;
  frc::TimeInterpolatableBuffer<FlywheelState> m_flywheelBuffer;

  template <typename T>
  void Log(const std::string& key, T val) {
    Logger::Instance().Log("RobotState/" + key, val);
  }
};
