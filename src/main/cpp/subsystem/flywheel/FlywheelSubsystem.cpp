// Team 5687 2026

#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "Constants.h"
#include <utility>

#include "subsystem/LoggedSubsystem.h"

FlywheelSubsystem::FlywheelSubsystem(std::unique_ptr<FlywheelIO> io)
    : LoggedSubsystem("Flywheel"), m_io(std::move(io)) {}

void FlywheelSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);
}

FlywheelState FlywheelSubsystem::GetFlywheelState() const {
  FlywheelState state;
  state.timestamp = m_inputs.timestamp;
  state.velocity = (m_inputs.motorVelocity / Constants::Flywheel::kGearRatio) *
                   (2.0 * std::numbers::pi * 1_rad) / 1_tr;
  state.acceleration =
      (m_inputs.motorAcceleration / Constants::Flywheel::kGearRatio) *
      (2.0 * std::numbers::pi * 1_rad) / 1_tr;

  return state;
}

void FlywheelSubsystem::SetRPM(units::revolutions_per_minute_t desiredRPMLeft, units::revolutions_per_minute_t desiredRPMRight) {
  m_desiredRPMLeft = desiredRPMLeft;
  m_desiredRPMRight = desiredRPMRight;
  m_io->SetFlywheelRPM(m_desiredRPMLeft, m_desiredRPMRight);
};

void FlywheelSubsystem::LogTelemetry() {
  Log("Desired RPM Left", m_desiredRPMLeft.value());
  Log("Flywheel Velocity Left", m_inputs.leftFlywheelVelocity.value());

  Log("Desired RPM Right", m_desiredRPMRight.value());
  Log("Flywheel Velocity Right", m_inputs.rightFlywheelVelocity.value());
}
