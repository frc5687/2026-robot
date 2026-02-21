// Team 5687 2026

#include "subsystem/kicker/KickerSubsystem.h"

#include "Constants.h"

KickerSubsystem::KickerSubsystem(std::unique_ptr<KickerIO> io)
    : LoggedSubsystem("Kicker"), m_io(std::move(io)) {}

void KickerSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void KickerSubsystem::SetVelocity(units::turns_per_second_t rps) {
  m_io->SetVelocity(rps);
}

void KickerSubsystem::Stop() { m_io->Stop(); }

void KickerSubsystem::UpdateInputs() { m_io->UpdateInputs(m_inputs); }

void KickerSubsystem::LogTelemetry() {
  Log("Velocity", m_inputs.motorVelocity.value());
  Log("Position", m_inputs.motorPosition.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("StatorCurrent", m_inputs.statorCurrent.value());
  Log("SupplyCurrent", m_inputs.supplyCurrent.value());
}
