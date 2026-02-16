// Team 5687 2026

#include "subsystem/turret/TurretSubsystem.h"

#include <numbers>
#include <utility>

#include "Constants.h"
#include "RobotState.h"

TurretSubsystem::TurretSubsystem(std::unique_ptr<TurretIO> io)
    : LoggedSubsystem("Turret"), m_io(std::move(io)) {}

void TurretSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);
  RobotState::Instance().AddTurretObservation(GetTurretState());
}

void TurretSubsystem::SetAngle(units::radian_t desiredAngle) {
  m_desiredAngle = desiredAngle;
  m_io->SetTurretAngle(m_desiredAngle);
}

TurretState TurretSubsystem::GetTurretState() {
  TurretState state;
  state.timestamp = m_inputs.timestamp;

  state.angle = (m_inputs.motorPosition / Constants::Turret::kGearRatio) *
                (2.0 * std::numbers::pi * 1_rad) / 1_tr;

  state.velocity = (m_inputs.motorVelocity / Constants::Turret::kGearRatio) *
                   (2.0 * std::numbers::pi * 1_rad) / 1_tr;

  state.acceleration =
      (m_inputs.motorAcceleration / Constants::Turret::kGearRatio) *
      (2.0 * std::numbers::pi * 1_rad) / 1_tr;

  state.torque = m_inputs.motorTorque * Constants::Turret::kGearRatio;

  return state;
}

void TurretSubsystem::LogTelemetry() {
  Log("Desired Angle", m_desiredAngle.value());
  Log("Motor Position", m_inputs.motorPosition.value());
  Log("Motor Velocity", m_inputs.motorVelocity.value());
  Log("Angle", GetTurretState().angle.value());
  Log("Velocity", GetTurretState().velocity.value());
  Log("Acceleration", GetTurretState().acceleration.value());
  Log("Torque", GetTurretState().torque.value());
}
