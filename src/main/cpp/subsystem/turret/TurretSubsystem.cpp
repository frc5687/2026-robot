#include "subsystem/turret/TurretSubsystem.h"

TurretSubsystem::TurretSubsystem(std::unique_ptr<TurretIO> io) :
    LoggedSubsystem("Turret"),
    m_io(std::move(io)) {
}

void TurretSubsystem::UpdateInputs() {
    m_io->UpdateInputs(m_inputs);
}

void TurretSubsystem::SetAngle(units::radian_t desiredAngle) {
    m_desiredAngle = desiredAngle;
}

void TurretSubsystem::LogTelemetry() {
  Log("Desired Angle", m_desiredAngle.value());
  Log("Motor Position", m_inputs.motorPosition.value());
  Log("Motor Velocity", m_inputs.motorVelocity.value());
  Log("Angle", m_inputs.angle.value());
  Log("AngularVelocity", m_inputs.angularVelocity.value());
}
