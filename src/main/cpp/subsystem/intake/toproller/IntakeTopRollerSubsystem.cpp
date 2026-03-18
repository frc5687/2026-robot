// Team 5687 2026

#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"

#include <units/math.h>

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
  Log("Current/Leader/Supply", m_inputs.supplyCurrent.value());
  Log("Current/Follower1/Supply", m_inputs.followerSupplyCurrent.value());
}

units::ampere_t IntakeTopRollerSubsystem::GetElectricalCurrentDraw() const {
  return m_inputs.supplyCurrent + m_inputs.followerSupplyCurrent;
}

units::watt_t IntakeTopRollerSubsystem::GetElectricalPowerDraw() const {
  const auto leaderPower = units::math::abs(m_inputs.supplyCurrent) *
                          m_inputs.appliedVolts;
  const auto followerPower =
      units::math::abs(m_inputs.followerSupplyCurrent) *
      m_inputs.followerAppliedVolts;
  return leaderPower + followerPower;
}
