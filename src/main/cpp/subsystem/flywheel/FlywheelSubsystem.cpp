#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/LoggedSubsystem.h"
#include "subsystem/flywheel/FlywheelIO.h"
#include "units/angular_velocity.h"
#include <memory>

FlywheelSubsystem::FlywheelSubsystem(std::unique_ptr<FlywheelIO> io) :
  LoggedSubsystem("Flywheel"),
  m_io(std::move(io)) {}

void FlywheelSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);
  m_inputs.rightFlywheelVelocity = m_filter.Calculate(m_inputs.rightFlywheelVelocity);
};

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
