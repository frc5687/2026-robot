// Team 5687 2026

#include "subsystem/flywheel/SimFlywheelIO.h"

#include <frc/Timer.h>
#include <units/math.h>

#include <numbers>

#include "Constants.h"

// ════════════════════════════════════════════════════════════════════════
// SimMotor — one flywheel motor + its physics sim
// ════════════════════════════════════════════════════════════════════════

SimFlywheelIO::SimMotor::SimMotor(const frc::LinearSystem<1, 1, 1> &plant,
                                  const frc::DCMotor &motor, double gearRatio,
                                  double kP, double kD,
                                  decltype(1_V / 1_rad_per_s) kVff,
                                  units::volt_t kSff,
                                  decltype(1_V / 1_rad_per_s_sq) kAff)
    // WPILib 2025+: FlywheelSim(plant, gearbox) — NO gearing param.
    // Gearing is computed internally from the plant matrices and motor model.
    : sim(plant, motor), pid(kP, 0.0, kD), feedforward(kSff, kVff, kAff),
      gearing(gearRatio) {}

units::volt_t SimFlywheelIO::SimMotor::CalculateClosedLoop() {
  if (setpoint.value() <= 0)
    return 0_V;

  // Convert motor RPS setpoint → mechanism rad/s for the controller.
  // The controller operates in mechanism-level units because that's
  // what the sim reports.
  //
  //   mechanism_rad_per_s = motor_rps * (2π rad/turn) / gear_ratio
  //
  auto mechanismSetpointRad = units::radians_per_second_t{
      setpoint.value() * 2.0 * std::numbers::pi / gearing};

  auto currentMechanismRad = GetMechanismVelocity();

  // Feedforward: holds velocity at steady state
  auto ff = feedforward.Calculate(mechanismSetpointRad);

  // PID: corrects for disturbances (ball contact, model error)
  auto fb = units::volt_t{
      pid.Calculate(currentMechanismRad.value(), mechanismSetpointRad.value())};

  auto total = ff + fb;
  total = units::math::max(total, -12_V);
  total = units::math::min(total, 12_V);
  return total;
}

void SimFlywheelIO::SimMotor::Update(units::second_t dt,
                                     units::volt_t voltage) {
  lastAppliedVoltage = voltage;

  // SetInputVoltage takes motor voltage — the sim handles the
  // gearing internally when computing mechanism dynamics.
  sim.SetInputVoltage(voltage);
  sim.Update(dt);

  // Integrate motor position from motor velocity
  motorPosition += units::turn_t{GetMotorVelocity().value() * dt.value()};
}

units::radians_per_second_t
SimFlywheelIO::SimMotor::GetMechanismVelocity() const {
  // GetAngularVelocity() returns MECHANISM velocity (after gearing)
  return sim.GetAngularVelocity();
}

units::turns_per_second_t SimFlywheelIO::SimMotor::GetMotorVelocity() const {
  // Convert mechanism rad/s → motor turns/s
  //
  //   motor_rps = mechanism_rad_per_s * gear_ratio / (2π rad/turn)
  //
  auto mechanismRadPerSec = sim.GetAngularVelocity();
  return units::turns_per_second_t{mechanismRadPerSec.value() * gearing /
                                   (2.0 * std::numbers::pi)};
}

units::ampere_t SimFlywheelIO::SimMotor::GetCurrentDraw() const {
  // GetCurrentDraw() already returns motor-level current
  return sim.GetCurrentDraw();
}

// ════════════════════════════════════════════════════════════════════════
// Factory
// ════════════════════════════════════════════════════════════════════════

SimFlywheelIO::SimMotor SimFlywheelIO::MakeSimMotor() {
  auto motor = frc::DCMotor::KrakenX60FOC(1);

  // FlywheelSystem bakes gearing into the state-space matrices.
  // The plant state (angular velocity) is in mechanism-level rad/s.
  auto plant = frc::LinearSystemId::FlywheelSystem(
      motor, Constants::Flywheel::kInertia, Constants::Flywheel::kGearRatio);

  // Feedforward gains in mechanism rad/s (matching the sim's output units)
  auto kVff = decltype(1_V / 1_rad_per_s){Constants::Flywheel::kSimKv};
  auto kSff = units::volt_t{Constants::Flywheel::kSimKs};
  auto kAff = decltype(1_V / 1_rad_per_s_sq){Constants::Flywheel::kSimKa};

  return SimMotor(plant, motor, Constants::Flywheel::kGearRatio,
                  Constants::Flywheel::kSimP, Constants::Flywheel::kSimD, kVff,
                  kSff, kAff);
}

// ════════════════════════════════════════════════════════════════════════
// Construction
// ════════════════════════════════════════════════════════════════════════

SimFlywheelIO::SimFlywheelIO()
    : m_left(MakeSimMotor()), m_right(MakeSimMotor()) {}

// ════════════════════════════════════════════════════════════════════════
// FlywheelIO Interface
// ════════════════════════════════════════════════════════════════════════

void SimFlywheelIO::UpdateInputs(FlywheelIOInputs &inputs) {
  constexpr auto dt = 20_ms;

  if (m_characterizing) {
    // Open-loop: raw voltage straight to motor (SysId characterization)
    m_left.Update(dt, m_characterizationVoltage);
    m_right.Update(dt, m_characterizationVoltage);
  } else {
    // Closed-loop: feedforward + PID computes voltage (normal operation)
    m_left.Update(dt, m_left.CalculateClosedLoop());
    m_right.Update(dt, m_right.CalculateClosedLoop());
  }

  // ── Report raw MOTOR values (before gear ratio) ─────────────────────
  // This matches what the real CTREFlywheelIO reads from the TalonFX.
  inputs.leftMotorPosition = m_left.motorPosition;
  inputs.rightMotorPosition = m_right.motorPosition;
  inputs.leftMotorVelocity = m_left.GetMotorVelocity();
  inputs.rightMotorVelocity = m_right.GetMotorVelocity();

  inputs.leftAppliedVolts = m_left.lastAppliedVoltage;
  inputs.rightAppliedVolts = m_right.lastAppliedVoltage;

  inputs.leftStatorCurrent = m_left.GetCurrentDraw();
  inputs.rightStatorCurrent = m_right.GetCurrentDraw();
  inputs.leftSupplyCurrent = m_left.GetCurrentDraw();
  inputs.rightSupplyCurrent = m_right.GetCurrentDraw();

  inputs.timestamp = units::second_t{frc::Timer::GetFPGATimestamp().value()};
}

void SimFlywheelIO::SetMotorVelocity(units::turns_per_second_t leftRPS,
                                     units::turns_per_second_t rightRPS) {
  m_characterizing = false;
  m_left.setpoint = leftRPS;
  m_right.setpoint = rightRPS;
}

void SimFlywheelIO::SetVoltage(units::volt_t voltage) {
  m_characterizing = true;
  m_characterizationVoltage = voltage;
}
