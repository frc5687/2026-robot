// Team 5687 2026

#include "RobotState.h"

#include <ctime>

#include "frc/Timer.h"
#include "subsystem/flywheel/FlywheelState.h"
#include "subsystem/turret/TurretState.h"
#include "units/time.h"

RobotState::RobotState()
    : m_driveBuffer(BUFFER_TIME, OdometryData::Interpolate),
      m_turretBuffer(BUFFER_TIME, TurretState::Interpolate),
      m_flywheelBuffer(BUFFER_TIME, FlywheelState::Interpolate),
      m_hoodBuffer(BUFFER_TIME, HoodState::Interpolate) {}

void RobotState::AddDriveObservation(const OdometryData& state) {
  m_driveBuffer.AddSample(state.timestamp, state);
}

void RobotState::AddTurretObservation(const TurretState& state) {
  m_turretBuffer.AddSample(state.timestamp, state);
}

void RobotState::AddFlywheelObservation(const FlywheelState& state) {
  m_flywheelBuffer.AddSample(state.timestamp, state);
}

void RobotState::AddHoodObservation(const HoodState& state) {
  m_hoodBuffer.AddSample(state.timestamp, state);
}

OdometryData RobotState::GetDriveState(units::second_t timestamp) {
    return GetState<OdometryData>(m_driveBuffer, timestamp);
}

TurretState RobotState::GetTurretState(units::second_t timestamp) {
    return GetState<TurretState>(m_turretBuffer, timestamp);
}

FlywheelState RobotState::GetFlywheelState(units::second_t timestamp) {
  return GetState<FlywheelState>(m_flywheelBuffer, timestamp);
}

HoodState RobotState::GetHoodState(units::second_t timestamp) {
    return GetState<HoodState>(m_hoodBuffer, timestamp);
}

void RobotState::LogState(units::second_t timestamp) {
  TurretState turretState = GetTurretState(timestamp);
  Log("Turret Angle", turretState.angle.value());
  Log("Turret Velocity", turretState.velocity.value());
  Log("Turret Acceleration", turretState.acceleration.value());
  Log("Turret torque", turretState.torque.value());
}
