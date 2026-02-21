// Team 5687 2026

#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"

#include "Constants.h"

IntakeTopRollerSubsystem::IntakeTopRollerSubsystem(
    std::unique_ptr<IntakeTopRollerIO> io)
    : LoggedSubsystem("IntakeTopRoller"), m_io(std::move(io)) {}

void IntakeTopRollerSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void IntakeTopRollerSubsystem::SetVelocity(units::turns_per_second_t rps) {
  m_io->SetVelocity(rps);
}

void IntakeTopRollerSubsystem::Stop() { m_io->Stop(); }

void IntakeTopRollerSubsystem::UpdateInputs() { m_io->UpdateInputs(m_inputs); }

void IntakeTopRollerSubsystem::LogTelemetry() {
  Log("Velocity", m_inputs.motorVelocity.value());
  Log("Position", m_inputs.motorPosition.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("StatorCurrent", m_inputs.statorCurrent.value());
  Log("SupplyCurrent", m_inputs.supplyCurrent.value());
}
