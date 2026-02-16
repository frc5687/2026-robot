// Team 5687 2026

#include "subsystem/flywheel/FlywheelSubsystem.h"

#include <utility>

#include "Constants.h"
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

void FlywheelSubsystem::LogTelemetry() {
  Log("Motor Velocity", m_inputs.motorVelocity.value());
  // Log("Flywheel Velocity", m_inputs.flywheelVelocity.value());
  Log("Timestamp", m_inputs.timestamp.value());
}
