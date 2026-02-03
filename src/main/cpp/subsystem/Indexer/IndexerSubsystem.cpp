#include "subsystem/Indexer/IndexerSubsystem.h"

IndexerSubsystem::IndexerSubsystem(std::unique_ptr<IndexerIO> io) :
    LoggedSubsystem("Indexer"),
    m_io(std::move(io)) {
}

void IndexerSubsystem::UpdateInputs() {
    m_io->UpdateInputs(m_inputs);

}


void IndexerSubsystem::SetVoltage(units::volt_t voltage) {
    m_io->SetVoltage(voltage);
}

void IndexerSubsystem::LogTelemetry() {
  Log("AngularVelocity", m_inputs.MotorVelocity.value());
}