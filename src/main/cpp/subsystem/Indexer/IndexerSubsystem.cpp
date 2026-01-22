#include "subsystem/Indexer/IndexerSubsystem.h"
#include <cmath>

IndexerSubsystem::IndexerSubsystem(std::unique_ptr<IndexerIO> io) :
    LoggedSubsystem("Indexer"),
    m_io(std::move(io)) {
}

void IndexerSubsystem::UpdateInputs() {
    m_io->UpdateInputs(m_inputs);

}

// void IndexerSubsystem::SetAngle(units::radian_t desiredAngle) {
//     m_desiredAngle = desiredAngle;
//     m_io->SetTurretAngle(m_desiredAngle);
// }

void IndexerSubsystem::SetVoltage(units::volt_t voltage) {
    m_io->SetMotorVoltage(voltage);
}

void IndexerSubsystem::LogTelemetry() {
//   Log("Desired Angle", m_desiredAngle.value());
//   Log("Motor Position", m_inputs.motorPosition.value());
//   Log("Motor Velocity", m_inputs.motorVelocity.value());
//   Log("Angle", m_inputs.angle.value());
  Log("AngularVelocity", m_inputs.MotorVelocity.value());
}