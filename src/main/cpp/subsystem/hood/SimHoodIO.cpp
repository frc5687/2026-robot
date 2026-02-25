// Team 5687 2026

#include "subsystem/hood/SimHoodIO.h"

#include <frc/Timer.h>
#include <units/math.h>

#include <numbers>

#include "Constants.h"

using namespace Constants::Hood;

SimHoodIO::SimHoodIO()
    : m_armSim(kMotor, kGearRatio, kMoi, kArmLength,
               kMinAngle * (2.0 * std::numbers::pi * 1_rad / 1_tr),
               kMaxAngle * (2.0 * std::numbers::pi * 1_rad / 1_tr), true,
               0_rad),
      m_pid(SimPID::kP, SimPID::kI, SimPID::kD) {}

units::volt_t SimHoodIO::CalculateClosedLoop() {
  auto setpointRad = (m_positionSetpoint / kGearRatio) *
                     (2.0 * std::numbers::pi * 1_rad / 1_tr);
  auto currentRad = m_armSim.GetAngle();

  auto fb =
      units::volt_t{m_pid.Calculate(currentRad.value(), setpointRad.value())};
  return units::math::max(units::math::min(fb, 12_V), -12_V);
}

void SimHoodIO::UpdateInputs(HoodIOInputs &inputs) {
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

  m_armSim.SetInputVoltage(voltage);
  m_armSim.Update(dt);

  auto armAngle = m_armSim.GetAngle();
  auto armVelocity = m_armSim.GetVelocity();

  inputs.motorPosition =
      (armAngle * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;
  inputs.motorVelocity =
      (armVelocity * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;
  inputs.appliedVolts = voltage;
  inputs.statorCurrent = m_armSim.GetCurrentDraw();
  inputs.supplyCurrent = m_armSim.GetCurrentDraw();
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimHoodIO::SetPosition(units::turn_t position) {
  m_mode = Mode::kPosition;
  m_positionSetpoint = position;
}

void SimHoodIO::SetVoltage(units::volt_t voltage) {
  m_mode = Mode::kVoltage;
  m_voltageCommand = voltage;
}

void SimHoodIO::ZeroPosition() {}

void SimHoodIO::Stop() {
  m_mode = Mode::kStopped;
  m_voltageCommand = 0_V;
}
