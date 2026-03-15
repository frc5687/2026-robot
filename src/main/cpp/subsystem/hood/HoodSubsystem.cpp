// Team 5687 2026

#include "subsystem/hood/HoodSubsystem.h"

#include <units/math.h>

#include <numbers>

#include "subsystem/hood/HoodConstants.h"
#include "RobotState.h"

using namespace Constants::Hood;

HoodSubsystem::HoodSubsystem(std::unique_ptr<HoodIO> io)
    : LoggedSubsystem("Hood"), m_io(std::move(io)) {}

void HoodSubsystem::SetPosition(units::radian_t mechanismAngle) {
  units::turn_t motorTurns{mechanismAngle.value() / (2.0 * std::numbers::pi) *
                           kGearRatio};
  m_io->SetPosition(motorTurns);
}

void HoodSubsystem::SetVoltage(units::volt_t voltage) {
  m_io->SetVoltage(voltage);
}

void HoodSubsystem::ZeroPosition() { m_io->ZeroPosition(); }

void HoodSubsystem::Stop() { m_io->Stop(); }

units::radian_t HoodSubsystem::GetPosition() const {
  return units::radian_t{(m_inputs.motorPosition / kGearRatio).value() * 2.0 *
                         std::numbers::pi};
}

bool HoodSubsystem::IsAtPosition(units::radian_t target) const {
  return units::math::abs(GetPosition() - target) < kAngleTolerance;
}

HoodState HoodSubsystem::GetHoodState() const {
  HoodState state;
  state.timestamp = m_inputs.timestamp;
  state.angle = units::radian_t{(m_inputs.motorPosition / kGearRatio).value() *
                                2.0 * std::numbers::pi};
  state.velocity = units::radians_per_second_t{
      (m_inputs.motorVelocity / kGearRatio).value() * 2.0 * std::numbers::pi};
  return state;
}

void HoodSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);
  RobotState::Instance().AddHoodObservation(GetHoodState());
}

void HoodSubsystem::LogTelemetry() {
  Log("MotorPosition", m_inputs.motorPosition.value());
  Log("MotorVelocity", m_inputs.motorVelocity.value());
  Log("AppliedVolts", m_inputs.appliedVolts.value());
  Log("StatorCurrent", m_inputs.statorCurrent.value());
  Log("SupplyCurrent", m_inputs.supplyCurrent.value());
  Log("MechanismAngleRad", GetPosition().value());
}
