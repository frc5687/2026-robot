// Team 5687 2026

#include "subsystem/kicker/SimKickerIO.h"

#include <frc/Timer.h>
#include <units/math.h>

#include <numbers>

#include "Constants.h"

using namespace Constants::Kicker;

SimKickerIO::SimKickerIO()
    : m_motorSim(
          frc::LinearSystemId::DCMotorSystem(kMotor, kInertia, kGearRatio),
          kMotor),
      m_pid(PID::kP, 0.0, PID::kD) {}

units::volt_t SimKickerIO::CalculateClosedLoop() {
  if (m_velocitySetpoint.value() <= 0)
    return 0_V;

  auto setpointRad = units::radians_per_second_t{
      m_velocitySetpoint.value() * 2.0 * std::numbers::pi / kGearRatio};
  auto currentRad = m_motorSim.GetAngularVelocity();

  auto fb =
      units::volt_t{m_pid.Calculate(currentRad.value(), setpointRad.value())};
  return units::math::max(units::math::min(fb, 12_V), -12_V);
}

void SimKickerIO::UpdateInputs(KickerIOInputs &inputs) {
  constexpr auto dt = 20_ms;

  units::volt_t voltage{0_V};
  switch (m_mode) {
  case Mode::kVoltage:
    voltage = m_voltageCommand;
    break;
  case Mode::kVelocity:
    voltage = CalculateClosedLoop();
    break;
  case Mode::kStopped:
    break;
  }

  m_motorSim.SetInputVoltage(voltage);
  m_motorSim.Update(dt);

  inputs.motorPosition = units::turn_t{m_motorSim.GetAngularPosition().value() *
                                       kGearRatio / (2.0 * std::numbers::pi)};
  inputs.motorVelocity =
      units::turns_per_second_t{m_motorSim.GetAngularVelocity().value() *
                                kGearRatio / (2.0 * std::numbers::pi)};
  inputs.appliedVolts = voltage;
  inputs.statorCurrent = m_motorSim.GetCurrentDraw();
  inputs.supplyCurrent = m_motorSim.GetCurrentDraw();
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimKickerIO::SetVoltage(units::volt_t voltage) {
  m_mode = Mode::kVoltage;
  m_voltageCommand = voltage;
}

void SimKickerIO::SetVelocity(units::turns_per_second_t rps) {
  m_mode = Mode::kVelocity;
  m_velocitySetpoint = rps;
}

void SimKickerIO::Stop() {
  m_mode = Mode::kStopped;
  m_voltageCommand = 0_V;
  m_velocitySetpoint = units::turns_per_second_t{0};
}
