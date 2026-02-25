// Team 5687 2026

#include "subsystem/floorroller/FloorRollerSubsystem.h"

#include <frc/Timer.h>

FloorRollerSubsystem::FloorRollerSubsystem(std::unique_ptr<FloorRollerIO> io)
    : LoggedSubsystem("FloorRoller"), m_io(std::move(io)) {}

void FloorRollerSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void FloorRollerSubsystem::SetVelocity(units::turns_per_second_t rps) {
  m_io->SetVelocity(rps);
}

void FloorRollerSubsystem::Stop() { m_io->Stop(); }

void FloorRollerSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);

  m_state.velocity = m_inputs.motorVelocity;
  m_state.timestamp = m_inputs.timestamp;
}

void FloorRollerSubsystem::LogTelemetry() {
  Log("Velocity", m_inputs.motorVelocity.value());
  Log("Position", m_inputs.motorPosition.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("StatorCurrent", m_inputs.statorCurrent.value());
  Log("SupplyCurrent", m_inputs.supplyCurrent.value());
}
