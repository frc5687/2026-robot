// Team 5687 2026

#include "subsystem/intake/SimIntakeIO.h"

#include <frc/Timer.h>
#include <frc/system/plant/LinearSystemId.h>

#include <numbers>

#include "Constants.h"

using namespace Constants::Intake;

SimIntakeIO::SimIntakeIO() {}
// :
// m_sim(
//      frc::LinearSystem::FlywheelSystem(kMotor, kInertia, kGearRatio),
//      kMotor) {}

void SimIntakeIO::UpdateInputs(IntakeIOInputs &inputs) {
  // m_sim.SetInputVoltage(m_voltage);
  // m_sim.Update(20_ms);

  // // FlywheelSim reports mechanism rad/s → convert to motor RPS
  // inputs.motorVelocity = units::turns_per_second_t{
  //     m_sim.GetAngularVelocity().value() * kGearRatio /
  //     (2.0 * std::numbers::pi)};

  // inputs.appliedVolts = m_voltage;
  // inputs.statorCurrent = m_sim.GetCurrentDraw();
  // inputs.supplyCurrent = m_sim.GetCurrentDraw();
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimIntakeIO::SetVoltage(units::volt_t voltage) { m_voltage = voltage; }

void SimIntakeIO::Stop() { m_voltage = 0_V; }
