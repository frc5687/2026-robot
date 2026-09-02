// Team 5687 2026

#include "RobotState.h"

#include "subsystem/flywheel/FlywheelState.h"

RobotState::RobotState()
    : m_driveBuffer(BUFFER_TIME, OdometryData::Interpolate),
      m_flywheelBuffer(BUFFER_TIME, FlywheelState::Interpolate),
      m_hoodBuffer(BUFFER_TIME, HoodState::Interpolate) {}

void RobotState::AddDriveObservation(const OdometryData &state) {
  m_driveBuffer.AddSample(state.timestamp, state);
  m_lastDriveState = state;
}

void RobotState::AddFlywheelObservation(const FlywheelState &state) {
  m_flywheelBuffer.AddSample(state.timestamp, state);
  m_lastFlywheelState = state;
}

void RobotState::AddHoodObservation(const HoodState &state) {
  m_hoodBuffer.AddSample(state.timestamp, state);
  m_lastHoodState = state;
}

OdometryData RobotState::GetDriveState(units::second_t timestamp) {
  return GetState<OdometryData>(m_driveBuffer, m_lastDriveState, timestamp);
}

FlywheelState RobotState::GetFlywheelState(units::second_t timestamp) {
  return GetState<FlywheelState>(m_flywheelBuffer, m_lastFlywheelState,
                                 timestamp);
}

HoodState RobotState::GetHoodState(units::second_t timestamp) {
  return GetState<HoodState>(m_hoodBuffer, m_lastHoodState, timestamp);
}
