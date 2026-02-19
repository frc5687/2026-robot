// Team 5687 2026

#include "subsystem/turret/SimTurretIO.h"

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "Constants.h"
#include "frc/Timer.h"
#include "frc/system/plant/LinearSystemId.h"

using namespace Constants::Turret;

SimTurretIO::SimTurretIO()
    : m_turretSim(
          frc::LinearSystemId::DCMotorSystem(kMotor, kInertia, kGearRatio),
          kMotor, {0.001, 0.001}), // Standard deviations for noise
      m_controller(kP, kI, kD) {}

void SimTurretIO::UpdateInputs(TurretIOInputs &inputs) {
  constexpr auto kDt = 20_ms;

  m_turretSim.SetInputVoltage(units::volt_t{
      m_controller.Calculate(m_turretSim.GetAngularPosition().value())});
  m_turretSim.Update(kDt);

  auto turretPosition = m_turretSim.GetAngularPosition();
  auto turretVelocity = m_turretSim.GetAngularVelocity();
  auto turretAccel = m_turretSim.GetAngularAcceleration();

  inputs.motorPosition =
      (turretPosition * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;

  inputs.motorVelocity =
      (turretVelocity * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;

  inputs.motorAcceleration =
      (turretAccel * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;

  inputs.motorCurrent = m_turretSim.GetCurrentDraw();

  inputs.motorTorque = m_turretSim.GetCurrentDraw() * kMotor.Kt;

  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimTurretIO::SetTurretAngle(units::radian_t angle) {
  m_controller.SetSetpoint(angle.value());
}
