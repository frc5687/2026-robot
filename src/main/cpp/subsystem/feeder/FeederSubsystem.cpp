// Team 5687 2026

#include "subsystem/feeder/FeederSubsystem.h"

#include <frc/Timer.h>
#include <units/math.h>

#include "frc2/command/sysid/SysIdRoutine.h"
#include "units/current.h"

using namespace frc2::sysid;

FeederSubsystem::FeederSubsystem(std::unique_ptr<FeederIO> io)
    : LoggedSubsystem("Feeder"), m_io(std::move(io)),
      m_sysIdRoutine{
          frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                              nullptr},
          Mechanism{[this](units::volt_t v) { SysIdDrive(v); },
                    [this](frc::sysid::SysIdRoutineLog *log) { SysIdLog(log); },
                    this, "Feeder"}} {}

void FeederSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void FeederSubsystem::SetVelocity(units::turns_per_second_t rps) {
  m_io->SetVelocity(rps);
}

void FeederSubsystem::SetPosition(units::turn_t position) {
  m_io->SetPosition(position);
}

void FeederSubsystem::Stop() { m_io->Stop(); }

units::turn_t FeederSubsystem::GetPosition() const {
  return m_inputs.motorPosition;
}

bool FeederSubsystem::NeedsIndexing() const { return m_needsIndexing; }

void FeederSubsystem::SetIndexed() { m_needsIndexing = false; }

void FeederSubsystem::ClearIndexed() { m_needsIndexing = true; }

frc2::CommandPtr FeederSubsystem::SysIdQuasistatic(Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr FeederSubsystem::SysIdDynamic(Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}

void FeederSubsystem::SysIdDrive(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}
void FeederSubsystem::SetCurrent(units::ampere_t current) {
  m_io->SetCurrent(current);
}

bool FeederSubsystem::isFuelDetected(){
  return m_inputs.fuelDetected;
}

void FeederSubsystem::SysIdLog(frc::sysid::SysIdRoutineLog *log) {
  // log->Motor("feeder-leader")
  //     .voltage(m_inputs.appliedVolts)
  //     .position(m_inputs.motorPosition)
  //     .velocity(m_inputs.motorVelocity);
}

void FeederSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);

  m_state.velocity = m_inputs.motorVelocity;
  m_state.timestamp = m_inputs.timestamp;
}

void FeederSubsystem::LogTelemetry() {
  const auto leaderPower =
      units::math::abs(m_inputs.supplyCurrent) * m_inputs.appliedVolts;
  const auto followerPower =
      units::math::abs(m_inputs.followerSupplyCurrent) *
      m_inputs.followerAppliedVolts;

  Log("Velocity", m_inputs.motorVelocity.value());
  Log("Position", m_inputs.motorPosition.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("Current/Leader/Stator", m_inputs.statorCurrent.value());
  Log("Current/Leader/Supply", m_inputs.supplyCurrent.value());
  Log("Current/Follower/Supply", m_inputs.followerSupplyCurrent.value());
  Log("Current/Follower/Stator", m_inputs.followerStatorCurrent.value());
  Log("Voltage/Follower/Applied", m_inputs.followerAppliedVolts.value());
  Log("Power/Leader", leaderPower.value());
  Log("Power/Follower", followerPower.value());
  Log("Current/Total/Supply", GetElectricalCurrentDraw().value());
  Log("Power/Total", (leaderPower + followerPower).value());
  Log("isFuelDetected", isFuelDetected());
}

units::ampere_t FeederSubsystem::GetElectricalCurrentDraw() const {
  return m_inputs.supplyCurrent + m_inputs.followerSupplyCurrent;
}

units::watt_t FeederSubsystem::GetElectricalPowerDraw() const {
  const auto leaderPower =
      units::math::abs(m_inputs.supplyCurrent) * m_inputs.appliedVolts;
  const auto followerPower =
      units::math::abs(m_inputs.followerSupplyCurrent) *
      m_inputs.followerAppliedVolts;

  return leaderPower + followerPower;
}
