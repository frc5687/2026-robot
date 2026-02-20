// Team 5687 2026

#pragma once

#include <frc/Timer.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <units/time.h>

#include <string>

#include "subsystem/drive/OdometryData.h"
#include "subsystem/flywheel/FlywheelState.h"
#include "subsystem/hood/HoodState.h"
#include "utils/Logger.h"

// This will store the entire state of subsystems we care about, this includes
// Transforms between systems, positional offsets from base, etc.

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

  void LogState(units::second_t timestamp);
  bool IsBallIndexing() const { return m_indexerBallDetected; }
  void SetBallIndexingEvent(bool isIndexing) {
    m_indexerBallDetected = isIndexing;
  }

private:
  RobotState();
  frc::TimeInterpolatableBuffer<OdometryData> m_driveBuffer;
  frc::TimeInterpolatableBuffer<FlywheelState> m_flywheelBuffer;
  frc::TimeInterpolatableBuffer<HoodState> m_hoodBuffer;

  bool m_indexerBallDetected;

  template <typename T> void Log(const std::string &key, T val) {
    Logger::Instance().Log("RobotState/" + key, val);
  }

  template <typename T>
  T GetState(const frc::TimeInterpolatableBuffer<T> &buffer,
             units::second_t timestamp) {
    auto currentTime = frc::Timer::GetFPGATimestamp();
    // We are looking to the future, extrapolate state
    if (currentTime < timestamp) {
      return buffer.Sample(timestamp).value().Extrapolate(timestamp -
                                                          currentTime);
    }
    // Otherwise use interpolation
    return buffer.Sample(timestamp).value();
  }
};
