// Team 5687 2026

#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"

#include "Constants.h"

IntakeBottomRollerSubsystem::IntakeBottomRollerSubsystem(
    std::unique_ptr<IntakeBottomRollerIO> io)
    : LoggedSubsystem("IntakeBottomRoller"), m_io(std::move(io)) {}

void IntakeBottomRollerSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void IntakeBottomRollerSubsystem::SetVelocity(units::turns_per_second_t rps) {
  m_io->SetVelocity(rps);
}

void IntakeBottomRollerSubsystem::Stop() { m_io->Stop(); }

void IntakeBottomRollerSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);
}

void IntakeBottomRollerSubsystem::LogTelemetry() {
  Log("Velocity", m_inputs.motorVelocity.value());
  Log("Position", m_inputs.motorPosition.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("StatorCurrent", m_inputs.statorCurrent.value());
  Log("SupplyCurrent", m_inputs.supplyCurrent.value());
}
