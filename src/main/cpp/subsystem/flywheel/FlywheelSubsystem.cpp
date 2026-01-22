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
};

void FlywheelSubsystem::SetRPM(units::revolutions_per_minute_t desiredRPM) {
    m_desiredRPM = desiredRPM;
    m_io->SetFlywheelRPM(m_desiredRPM);
};

void FlywheelSubsystem::LogTelemetry() {
  Log("Desired RPM", m_desiredRPM.value());
  Log("Motor Velocity", m_inputs.motorVelocity.value());
  Log("Flywheel Velocity", m_inputs.flywheelVelocity.value());
}
