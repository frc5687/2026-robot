// Team 5687 2026

#include "subsystem/intake/IntakeSubsystem.h"

#include "Constants.h"
#include "RobotState.h"

using namespace Constants::Intake;

IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IntakeIO> io)
    : LoggedSubsystem("Indexer"), m_io(std::move(io)) {}

void IntakeSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}


void IntakeSubsystem::Stop() { m_io->Stop(); }

void IntakeSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);
}

void IntakeSubsystem::LogTelemetry() {
}
