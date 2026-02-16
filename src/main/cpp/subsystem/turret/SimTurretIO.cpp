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
          kMotor, {0.001, 0.001}),  // Standard deviations for noise
      m_controller(kP, kI, kD) {}

void SimTurretIO::UpdateInputs(TurretIOInputs& inputs) {
  constexpr auto kDt = 20_ms;

  // 1. Update Physics
  m_turretSim.SetInputVoltage(units::volt_t{
      m_controller.Calculate(m_turretSim.GetAngularPosition().value())});
  m_turretSim.Update(kDt);

  // 2. Read Sim State (These are in Radians at the Turret Shaft)
  auto turretPosition = m_turretSim.GetAngularPosition();   // Rad
  auto turretVelocity = m_turretSim.GetAngularVelocity();   // Rad/s
  auto turretAccel = m_turretSim.GetAngularAcceleration();  // Rad/s^2

  // 3. Convert to Motor State (Rotations for Inputs struct)
  // Position: Radians -> Rotations
  inputs.motorPosition =
      (turretPosition * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;

  // Velocity: Radians/s -> Rotations/s
  inputs.motorVelocity =
      (turretVelocity * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;

  // Acceleration: Radians/s^2 -> Rotations/s^2 (FIXED)
  // Previously this was sending Radians but Subsystem expected Rotations logic
  inputs.motorAcceleration =
      (turretAccel * kGearRatio) / (2.0 * std::numbers::pi * 1_rad) * 1_tr;

  // 4. Electrical
  // FIXED: Do NOT multiply current by gear ratio.
  // The motor draws the same current regardless of the gearbox attached to it.
  inputs.motorCurrent = m_turretSim.GetCurrentDraw();

  // FIXED: Do NOT multiply Torque by gear ratio here.
  // 'motorTorque' implies torque at the motor shaft. The Subsystem handles the
  // gearing multiplier. Note: We calculate torque from current (T = Kt * I)
  // because LinearSystemSim usually doesn't expose a direct GetMotorTorque()
  inputs.motorTorque = m_turretSim.GetCurrentDraw() * kMotor.Kt;

  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimTurretIO::SetTurretAngle(units::radian_t angle) {
  m_controller.SetSetpoint(angle.value());
}
