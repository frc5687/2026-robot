// Team 5687 2026

#include "RobotState.h"

#include <ctime>

#include "frc/Timer.h"
#include "subsystem/turret/TurretState.h"
#include "units/time.h"

RobotState::RobotState()
    : m_driveBuffer(BUFFER_TIME, OdometryData::Interpolate),
      m_turretBuffer(BUFFER_TIME, TurretState::Interpolate),
      m_flywheelBuffer(BUFFER_TIME, FlywheelState::Interpolate) {}

void RobotState::AddDriveObservation(const OdometryData& state) {
  m_driveBuffer.AddSample(state.timestamp, state);
}

void RobotState::AddTurretObservation(const TurretState& state) {
  m_turretBuffer.AddSample(state.timestamp, state);
}

void RobotState::AddFlywheelObservation(const FlywheelState& state) {
  m_flywheelBuffer.AddSample(state.timestamp, state);
}

OdometryData RobotState::GetDriveState(units::second_t timestamp) {
  auto current = frc::Timer::GetFPGATimestamp();
  if (current < timestamp) {
    return m_driveBuffer.Sample(timestamp).value().Extrapolate(timestamp -
                                                               current);
  }
  return m_driveBuffer.Sample(timestamp).value();
}

TurretState RobotState::GetTurretState(units::second_t timestamp) {
  auto current = frc::Timer::GetFPGATimestamp();
  if (current < timestamp) {
    return m_turretBuffer.Sample(timestamp).value().Extrapolate(timestamp -
                                                                current);
  }
  return m_turretBuffer.Sample(timestamp).value();
}

FlywheelState RobotState::GetFlywheelState(units::second_t timestamp) {
  auto current = frc::Timer::GetFPGATimestamp();
  if (current < timestamp) {
    return m_flywheelBuffer.Sample(timestamp).value().Extrapolate(timestamp -
                                                                  current);
  }
  return m_flywheelBuffer.Sample(timestamp).value();
}

void RobotState::LogState(units::second_t timestamp) {
  TurretState turretState = GetTurretState(timestamp);
  Log("Turret Angle", turretState.angle.value());
  Log("Turret Velocity", turretState.velocity.value());
  Log("Turret Acceleration", turretState.acceleration.value());
  Log("Turret torque", turretState.torque.value());
}
