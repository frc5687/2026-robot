// Team 5687 2026

#pragma once

#include <frc/Timer.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <units/time.h>

#include <optional>

#include "subsystem/drive/OdometryData.h"
#include "subsystem/flywheel/FlywheelState.h"
#include "subsystem/hood/HoodState.h"

// This will store the entire state of subsystems we care about, this includes
// Transforms between systems, positional offsets from base, etc.
//
// What do I need to do:
// Track pose of all subsystems
// I just pass pose from subsystem or only needed values? angle, velocity, etc?
class RobotState {
public:
  const units::second_t BUFFER_TIME = 1.5_s;

  static RobotState &Instance() {
    static RobotState inst;
    return inst;
  }
  void AddDriveObservation(const OdometryData &state);
  void AddFlywheelObservation(const FlywheelState &state);
  void AddHoodObservation(const HoodState &state);

  // Interpolated or exterpolated based on time, limit to BUFFER_TIME
  OdometryData GetDriveState(units::second_t timestamp);
  FlywheelState GetFlywheelState(units::second_t timestamp);
  HoodState GetHoodState(units::second_t timestamp);

private:
  RobotState();

  frc::TimeInterpolatableBuffer<OdometryData> m_driveBuffer;
  frc::TimeInterpolatableBuffer<FlywheelState> m_flywheelBuffer;
  frc::TimeInterpolatableBuffer<HoodState> m_hoodBuffer;
  std::optional<OdometryData> m_lastDriveState;
  std::optional<FlywheelState> m_lastFlywheelState;
  std::optional<HoodState> m_lastHoodState;

  template <typename T>
  T GetState(const frc::TimeInterpolatableBuffer<T> &buffer,
             const std::optional<T> &lastState, units::second_t timestamp) {
    auto sample = buffer.Sample(timestamp);
    if (!sample.has_value()) {
      return lastState.value_or(T{});
    }

    auto currentTime = frc::Timer::GetFPGATimestamp();
    // We are looking to the future, extrapolate state
    if (currentTime < timestamp) {
      return sample->Extrapolate(timestamp - currentTime);
    }
    // Otherwise use interpolation
    return *sample;
  }

};
