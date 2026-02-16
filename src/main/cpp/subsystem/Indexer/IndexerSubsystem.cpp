#include "subsystem/Indexer/IndexerSubsystem.h"
#include "subsystem/indexer/IndexerConstants.h"
#include "units/angular_velocity.h"

IndexerSubsystem::IndexerSubsystem(std::unique_ptr<IndexerIO> io) :
    LoggedSubsystem("Indexer"),
    m_io(std::move(io)) {
}

void IndexerSubsystem::UpdateInputs() {
    m_io->UpdateInputs(m_inputs);

}


void IndexerSubsystem::SetVoltage(units::volt_t voltage, units::angular_velocity::turns_per_second_t rpm) {
    m_io->SetVoltage(voltage, rpm);
}

void IndexerSubsystem::LogTelemetry() {
  Log("AngularVelocity", m_inputs.MotorVelocity.value() * Constants::Indexer::kGearRatio);
}