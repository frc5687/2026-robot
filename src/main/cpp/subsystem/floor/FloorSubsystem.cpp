// Team 5687 2026

#include "subsystem/floor/FloorSubsystem.h"

#include <units/math.h>

FloorSubsystem::FloorSubsystem(std::unique_ptr<FloorIO> io)
    : LoggedSubsystem("Floor"), m_io(std::move(io)) {}

void FloorSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void FloorSubsystem::SetVelocity(units::turns_per_second_t rps) {
  m_io->SetVelocity(rps);
}

void FloorSubsystem::Stop() { m_io->Stop(); }

void FloorSubsystem::UpdateInputs() { m_io->UpdateInputs(m_inputs); }

void FloorSubsystem::LogTelemetry() {
  Log("Velocity", m_inputs.motorVelocity.value());
  Log("Position", m_inputs.motorPosition.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("Current/Stator", m_inputs.statorCurrent.value());
  Log("Current/Supply", m_inputs.supplyCurrent.value());
}

units::ampere_t FloorSubsystem::GetElectricalCurrentDraw() const {
  return m_inputs.supplyCurrent;
}

units::watt_t FloorSubsystem::GetElectricalPowerDraw() const {
  return units::math::abs(m_inputs.supplyCurrent) * m_inputs.appliedVolts;
}
