// Team 5687 2026

#include "subsystem/flywheel/FlywheelSubsystem.h"

#include <units/math.h>

#include <numbers>

#include "Constants.h"
#include "RobotState.h"
#include "frc2/command/sysid/SysIdRoutine.h"

using namespace frc2::sysid;

FlywheelSubsystem::FlywheelSubsystem(std::unique_ptr<FlywheelIO> io)
    : LoggedSubsystem("Flywheel"), m_io(std::move(io))
//      m_sysIdRoutine{
//          frc2::sysid::Config{
//              Constants::Flywheel::kSysIdRampRate,
//              Constants::Flywheel::kSysIdStepVoltage,
//              Constants::Flywheel::kSysIdTimeout,
//              nullptr},
//          Mechanism{
//              [this](units::volt_t v) { SysIdDrive(v); },
//              [this](frc::sysid::SysIdRoutineLog* log) { SysIdLog(log); },
//              this,
//              "Flywheel"}}
{}

void FlywheelSubsystem::SetRPM(
    units::revolutions_per_minute_t desiredRPMLeft,
    units::revolutions_per_minute_t desiredRPMRight) {
  m_desiredRPMLeft = desiredRPMLeft;
  m_desiredRPMRight = desiredRPMRight;
  m_io->SetMotorVelocity(MechanismRPMToMotorRPS(desiredRPMLeft),
                         MechanismRPMToMotorRPS(desiredRPMRight));
}

bool FlywheelSubsystem::AtSetpoint() const {
  constexpr auto tolerance = Constants::Flywheel::kAtSetpointTolerance;
  return units::math::abs(m_filteredLeft - m_desiredRPMLeft) < tolerance &&
         units::math::abs(m_filteredRight - m_desiredRPMRight) < tolerance;
}

// frc2::CommandPtr FlywheelSubsystem::SysIdQuasistatic(
//     Direction direction) {
//   return m_sysIdRoutine.Quasistatic(direction);
// }
//
// frc2::CommandPtr FlywheelSubsystem::SysIdDynamic(
//     Direction direction) {
//   return m_sysIdRoutine.Dynamic(direction);
// }

void FlywheelSubsystem::SysIdDrive(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void FlywheelSubsystem::SysIdLog(frc::sysid::SysIdRoutineLog *log) {
  log->Motor("flywheel-left")
      .voltage(m_inputs.leftAppliedVolts)
      .position(m_inputs.leftMotorPosition)
      .velocity(m_inputs.leftMotorVelocity);

  log->Motor("flywheel-right")
      .voltage(m_inputs.rightAppliedVolts)
      .position(m_inputs.rightMotorPosition)
      .velocity(m_inputs.rightMotorVelocity);
}

void FlywheelSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);

  auto leftMechRPM = MotorRPSToMechanismRPM(m_inputs.leftMotorVelocity);
  auto rightMechRPM = MotorRPSToMechanismRPM(m_inputs.rightMotorVelocity);

  m_filteredLeft = m_filterLeft.Calculate(leftMechRPM);
  m_filteredRight = m_filterRight.Calculate(rightMechRPM);

  UpdateFlywheelState(m_leftState, m_inputs.leftMotorVelocity,
                      m_inputs.timestamp);
  UpdateFlywheelState(m_rightState, m_inputs.rightMotorVelocity,
                      m_inputs.timestamp);
  RobotState::Instance().AddFlywheelObservation(m_leftState);
}

void FlywheelSubsystem::LogTelemetry() {
  Log("Left/FilteredRPM", m_filteredLeft.value());
  Log("Right/FilteredRPM", m_filteredRight.value());
  Log("Left/RawRPM",
      MotorRPSToMechanismRPM(m_inputs.leftMotorVelocity).value());
  Log("Right/RawRPM",
      MotorRPSToMechanismRPM(m_inputs.rightMotorVelocity).value());
  Log("Left/SetpointRPM", m_desiredRPMLeft.value());
  Log("Right/SetpointRPM", m_desiredRPMRight.value());
  Log("Left/ErrorRPM", (m_desiredRPMLeft - m_filteredLeft).value());
  Log("Right/ErrorRPM", (m_desiredRPMRight - m_filteredRight).value());

  Log("Left/VelocityRadPerSec", m_leftState.velocity.value());
  Log("Right/VelocityRadPerSec", m_rightState.velocity.value());
  Log("Left/AccelRadPerSecSq", m_leftState.acceleration.value());
  Log("Right/AccelRadPerSecSq", m_rightState.acceleration.value());

  Log("Left/AppliedVolts", m_inputs.leftAppliedVolts.value());
  Log("Right/AppliedVolts", m_inputs.rightAppliedVolts.value());
  Log("Left/StatorCurrent", m_inputs.leftStatorCurrent.value());
  Log("Right/StatorCurrent", m_inputs.rightStatorCurrent.value());

  Log("AtSetpoint", AtSetpoint());
}

void FlywheelSubsystem::UpdateFlywheelState(
    FlywheelState &state, units::turns_per_second_t motorVelocity,
    units::second_t timestamp) {
  auto mechanismVelocity = MotorRPSToMechanismRadPerSec(motorVelocity);

  auto dt = timestamp - state.timestamp;
  if (dt.value() > 0.0 && dt.value() < 0.5) {
    state.acceleration = units::radians_per_second_squared_t{
        (mechanismVelocity - state.velocity).value() / dt.value()};
  }

  state.velocity = mechanismVelocity;
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
