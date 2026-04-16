// Team 5687 2026

#include "subsystem/intake/deployer/SimIntakeDeployerIO.h"

#include <frc/Timer.h>
#include <units/math.h>

#include <numbers>

#include "subsystem/intake/IntakeConstants.h"

using namespace Constants::IntakeDeployer;

SimIntakeDeployerIO::SimIntakeDeployerIO()
    : m_motorSim(
          frc::LinearSystemId::DCMotorSystem(kMotor, kInertia, kGearRatio),
          kMotor),
      m_pid(SimPID::kP, 0.0, SimPID::kD) {}

units::volt_t SimIntakeDeployerIO::CalculateClosedLoop() {
  auto fb = units::volt_t{
      m_pid.Calculate(m_position.value(), m_positionSetpoint.value())};
  return units::math::max(units::math::min(fb, 12_V), -12_V);
}

void SimIntakeDeployerIO::UpdateInputs(IntakeDeployerIOInputs &inputs) {
  constexpr auto dt = 20_ms;

  units::volt_t voltage{0_V};
  switch (m_mode) {
  case Mode::kPosition:
    voltage = CalculateClosedLoop();
    break;
  case Mode::kVoltage:
    voltage = m_voltageCommand;
    break;
  case Mode::kStopped:
    break;
  }

  m_motorSim.SetInputVoltage(voltage);
  m_motorSim.Update(dt);

  units::turns_per_second_t motorVelocity{
      m_motorSim.GetAngularVelocity().value() * kGearRatio /
      (2.0 * std::numbers::pi)};
  m_position += motorVelocity * dt;

  m_position = units::math::max(units::math::min(m_position, kForwardSoftLimit),
                                kReverseSoftLimit);

  inputs.motorPosition = m_position;
  inputs.motorVelocity = motorVelocity;
  inputs.appliedVolts = voltage;
  inputs.statorCurrent = m_motorSim.GetCurrentDraw();
  inputs.supplyCurrent = m_motorSim.GetCurrentDraw();
  inputs.forwardLimitHit = m_position >= kForwardSoftLimit;
  inputs.reverseLimitHit = m_position <= kReverseSoftLimit;
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimIntakeDeployerIO::SetPosition(units::turn_t position) {
  m_mode = Mode::kPosition;
  m_positionSetpoint = position;
}

void SimIntakeDeployerIO::SetVoltage(units::volt_t voltage) {
  m_mode = Mode::kVoltage;
  m_voltageCommand = voltage;
}

void SimIntakeDeployerIO::ZeroPosition() { m_position = 0_tr; }

void SimIntakeDeployerIO::Stop() {
  m_mode = Mode::kStopped;
  m_voltageCommand = 0_V;
}

void SimIntakeDeployerIO::SetCurrentLimits(units::ampere_t currentlimits) {}
