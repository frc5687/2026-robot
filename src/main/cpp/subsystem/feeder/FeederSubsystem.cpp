// Team 5687 2026

#include "subsystem/feeder/FeederSubsystem.h"

#include <frc/Timer.h>

FeederSubsystem::FeederSubsystem(std::unique_ptr<FeederIO> io)
    : LoggedSubsystem("Feeder"), m_io(std::move(io)) {}

void FeederSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void FeederSubsystem::SetVelocity(units::turns_per_second_t rps) {
  m_io->SetVelocity(rps);
}

void FeederSubsystem::Stop() { m_io->Stop(); }

void FeederSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);

  m_state.velocity = m_inputs.motorVelocity;
  m_state.timestamp = m_inputs.timestamp;
}

void FeederSubsystem::LogTelemetry() {
  Log("Velocity", m_inputs.motorVelocity.value());
  Log("Position", m_inputs.motorPosition.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("StatorCurrent", m_inputs.statorCurrent.value());
  Log("SupplyCurrent", m_inputs.supplyCurrent.value());
}
