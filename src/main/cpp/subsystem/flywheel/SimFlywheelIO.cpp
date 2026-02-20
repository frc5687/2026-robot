// Team 5687 2026

#include "subsystem/flywheel/SimFlywheelIO.h"

#include <frc/Timer.h>
#include <units/math.h>

#include <numbers>

#include "Constants.h"

SimFlywheelIO::SimMotor::SimMotor(const frc::LinearSystem<1, 1, 1> &plant,
                                  const frc::DCMotor &motor, double gearRatio,
                                  double kP, double kD,
                                  decltype(1_V / 1_rad_per_s) kVff,
                                  units::volt_t kSff,
                                  decltype(1_V / 1_rad_per_s_sq) kAff)

    : sim(plant, motor), pid(kP, 0.0, kD), feedforward(kSff, kVff, kAff),
      gearing(gearRatio) {}

units::volt_t SimFlywheelIO::SimMotor::CalculateClosedLoop() {
  if (setpoint.value() <= 0)
    return 0_V;

  auto mechanismSetpointRad = units::radians_per_second_t{
      setpoint.value() * 2.0 * std::numbers::pi / gearing};

  auto currentMechanismRad = GetMechanismVelocity();

  auto ff = feedforward.Calculate(mechanismSetpointRad);

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

  sim.SetInputVoltage(voltage);
  sim.Update(dt);
  motorPosition += units::turn_t{GetMotorVelocity().value() * dt.value()};
}

units::radians_per_second_t
SimFlywheelIO::SimMotor::GetMechanismVelocity() const {
  return sim.GetAngularVelocity();
}

units::turns_per_second_t SimFlywheelIO::SimMotor::GetMotorVelocity() const {
  auto mechanismRadPerSec = sim.GetAngularVelocity();
  return units::turns_per_second_t{mechanismRadPerSec.value() * gearing /
                                   (2.0 * std::numbers::pi)};
}

units::ampere_t SimFlywheelIO::SimMotor::GetCurrentDraw() const {
  return sim.GetCurrentDraw();
}

SimFlywheelIO::SimMotor SimFlywheelIO::MakeSimMotor() {
  auto motor = frc::DCMotor::KrakenX60FOC(1);

  auto plant = frc::LinearSystemId::FlywheelSystem(
      motor, Constants::Flywheel::kInertia, Constants::Flywheel::kGearRatio);

  auto kVff = decltype(1_V / 1_rad_per_s){Constants::Flywheel::kSimKv};
  auto kSff = units::volt_t{Constants::Flywheel::kSimKs};
  auto kAff = decltype(1_V / 1_rad_per_s_sq){Constants::Flywheel::kSimKa};

  return SimMotor(plant, motor, Constants::Flywheel::kGearRatio,
                  Constants::Flywheel::kSimP, Constants::Flywheel::kSimD, kVff,
                  kSff, kAff);
}

SimFlywheelIO::SimFlywheelIO()
    : m_left(MakeSimMotor()), m_right(MakeSimMotor()) {}

void SimFlywheelIO::UpdateInputs(FlywheelIOInputs &inputs) {
  constexpr auto dt = 20_ms;

  if (m_characterizing) {
    m_left.Update(dt, m_characterizationVoltage);
    m_right.Update(dt, m_characterizationVoltage);
  } else {
    m_left.Update(dt, m_left.CalculateClosedLoop());
    m_right.Update(dt, m_right.CalculateClosedLoop());
  }

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
