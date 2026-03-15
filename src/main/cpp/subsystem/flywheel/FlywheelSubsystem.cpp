// Team 5687 2026

#include "subsystem/flywheel/FlywheelSubsystem.h"

#include <algorithm>
#include <units/math.h>
#include <frc/Timer.h>

#include <numbers>

#include "subsystem/flywheel/FlywheelConstants.h"
#include "RobotState.h"
#include "frc2/command/sysid/SysIdRoutine.h"

using namespace frc2::sysid;

FlywheelSubsystem::FlywheelSubsystem(std::unique_ptr<FlywheelIO> io)
    : LoggedSubsystem("Flywheel"), m_io(std::move(io)),
      m_sysIdRoutine{
          frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                              nullptr},
          Mechanism{[this](units::volt_t v) { SysIdDrive(v); },
                    [this](frc::sysid::SysIdRoutineLog *log) { SysIdLog(log); },
                    this, "Flywheel"}} {}

void FlywheelSubsystem::SetRPM(units::revolutions_per_minute_t desiredRPM) {
  auto now = units::second_t{frc::Timer::GetFPGATimestamp().value()};
  m_desiredRPM = desiredRPM;

  const auto deltaRPM = units::math::abs(desiredRPM - m_commandedRPM);
  const bool spinup =
      units::math::abs(desiredRPM) > units::math::abs(m_commandedRPM);
  const bool shouldRamp =
      spinup && deltaRPM > Constants::Flywheel::kSpinupRampThreshold &&
      Constants::Flywheel::kSpinupRampDuration > 0_s;

  if (!shouldRamp) {
    m_targetRPM = desiredRPM;
    m_rampStartRPM = desiredRPM;
    m_commandedRPM = desiredRPM;
    m_rampStartTime = now;
  } else {
    if (units::math::abs(desiredRPM - m_targetRPM) >
        Constants::Flywheel::kSpinupRetargetTolerance) {
      m_rampStartRPM = m_commandedRPM;
      m_targetRPM = desiredRPM;
      m_rampStartTime = now;
    }

    auto elapsed = now - m_rampStartTime;
    double progress = std::clamp(
        elapsed.value() / Constants::Flywheel::kSpinupRampDuration.value(), 0.0,
        1.0);
    m_commandedRPM = units::revolutions_per_minute_t{
        m_rampStartRPM.value() +
        (m_targetRPM.value() - m_rampStartRPM.value()) * progress};
  }

  m_io->SetMotorVelocity(MechanismRPMToMotorRPS(m_commandedRPM));
}

bool FlywheelSubsystem::AtSetpoint() const {
  return units::math::abs(m_filteredRPM - m_desiredRPM) <
         Constants::Flywheel::kAtSetpointTolerance;
}

frc2::CommandPtr FlywheelSubsystem::SysIdQuasistatic(Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr FlywheelSubsystem::SysIdDynamic(Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}

void FlywheelSubsystem::SysIdDrive(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void FlywheelSubsystem::SysIdLog(frc::sysid::SysIdRoutineLog *log) {
  // log->Motor("flywheel-leader")
  //     .voltage(m_inputs.leaderAppliedVolts)
  //     .position(m_inputs.leaderMotorPosition)
  //     .velocity(m_inputs.leaderMotorVelocity);
}

void FlywheelSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);

  auto mechRPM = MotorRPSToMechanismRPM(m_inputs.leaderMotorVelocity);
  m_filteredRPM = m_filter.Calculate(mechRPM);

  UpdateFlywheelState(m_state, m_inputs.leaderMotorVelocity,
                      m_inputs.timestamp);
  RobotState::Instance().AddFlywheelObservation(m_state);
}

void FlywheelSubsystem::LogTelemetry() {
  Log("Leader/FilteredRPM", m_filteredRPM.value());
  Log("Leader/RawRPM",
      MotorRPSToMechanismRPM(m_inputs.leaderMotorVelocity).value());
  Log("Leader/SetpointRPM", m_desiredRPM.value());
  Log("Leader/CommandedSetpointRPM", m_commandedRPM.value());
  Log("Leader/ErrorRPM", (m_desiredRPM - m_filteredRPM).value());

  Log("Leader/VelocityRadPerSec", m_state.velocity.value());
  Log("Leader/AccelRadPerSecSq", m_state.acceleration.value());

  Log("Leader/AppliedVolts", m_inputs.leaderAppliedVolts.value());
  Log("Leader/StatorCurrent", m_inputs.leaderStatorCurrent.value());
  Log("Leader/SupplyCurrent", m_inputs.leaderSupplyCurrent.value());

  Log("Follower1/SupplyCurrent", m_inputs.follower1SupplyCurrent.value());
  Log("Follower2/SupplyCurrent", m_inputs.follower2SupplyCurrent.value());
  Log("Follower3/SupplyCurrent", m_inputs.follower3SupplyCurrent.value());

  Log("AtSetpoint", AtSetpoint());
}

void FlywheelSubsystem::UpdateFlywheelState(
    FlywheelState &state, units::turns_per_second_t motorVelocity,
    units::second_t timestamp) {
  auto mechVelocity = MotorRPSToMechanismRadPerSec(motorVelocity);
  auto dt = timestamp - state.timestamp;

  if (dt.value() > 0.0 && dt.value() < 0.5) {
    state.acceleration = units::radians_per_second_squared_t{
        (mechVelocity - state.velocity).value() / dt.value()};
  }

  state.velocity = mechVelocity;
  state.timestamp = timestamp;
}

units::turns_per_second_t
FlywheelSubsystem::MechanismRPMToMotorRPS(units::revolutions_per_minute_t rpm) {
  return units::turns_per_second_t{(rpm.value() / 60.0) *
                                   Constants::Flywheel::kGearRatio};
}

units::revolutions_per_minute_t
FlywheelSubsystem::MotorRPSToMechanismRPM(units::turns_per_second_t rps) {
  return units::revolutions_per_minute_t{
      (rps.value() / Constants::Flywheel::kGearRatio) * 60.0};
}

units::radians_per_second_t
FlywheelSubsystem::MotorRPSToMechanismRadPerSec(units::turns_per_second_t rps) {
  return units::radians_per_second_t{
      (rps.value() / Constants::Flywheel::kGearRatio) * 2.0 * std::numbers::pi};
}
