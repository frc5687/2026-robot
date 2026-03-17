// Team 5687 2026

#include "subsystem/feeder/FeederSubsystem.h"

#include <frc/Timer.h>

#include "frc2/command/sysid/SysIdRoutine.h"

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

void FeederSubsystem::Stop() { m_io->Stop(); }

frc2::CommandPtr FeederSubsystem::SysIdQuasistatic(Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr FeederSubsystem::SysIdDynamic(Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}

void FeederSubsystem::SysIdDrive(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
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
  Log("Velocity", m_inputs.motorVelocity.value());
  Log("Position", m_inputs.motorPosition.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("StatorCurrent", m_inputs.statorCurrent.value());
  Log("SupplyCurrent", m_inputs.supplyCurrent.value());
  Log("Current/Stator", m_inputs.statorCurrent.value());
  Log("Current/Supply", m_inputs.supplyCurrent.value());
  Log("Current/Leader/Stator", m_inputs.statorCurrent.value());
  Log("Current/Leader/Supply", m_inputs.supplyCurrent.value());
  Log("Current/Follower1/Stator", m_inputs.follower1StatorCurrent.value());
  Log("Current/Follower1/Supply", m_inputs.follower1SupplyCurrent.value());
  Log("Current/Follower2/Stator", m_inputs.follower2StatorCurrent.value());
  Log("Current/Follower2/Supply", m_inputs.follower2SupplyCurrent.value());
  Log("Current/Follower3/Stator", m_inputs.follower3StatorCurrent.value());
  Log("Current/Follower3/Supply", m_inputs.follower3SupplyCurrent.value());
}
