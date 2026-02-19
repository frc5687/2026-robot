// Team 5687 2026

#include "subsystem/indexer/SimIndexerIO.h"

#include <frc/Timer.h>
#include <units/math.h>

#include <numbers>

#include "Constants.h"

using namespace Constants::Indexer;

SimIndexerIO::SimIndexerIO()
    : m_feedSim(
          frc::LinearSystemId::DCMotorSystem(kMotor, kInertia, kGearRatio),
          kMotor),
      m_centerSim(
          frc::LinearSystemId::DCMotorSystem(kMotor, kInertia, kGearRatio),
          kMotor),
      m_pid(PID::kP, 0.0, PID::kD) {}

void SimIndexerIO::InjectBall() {
  m_ballPresent = true;
  m_ballTimer = 0_s;
}

units::volt_t SimIndexerIO::CalculateClosedLoop() {
  if (m_velocitySetpoint.value() <= 0)
    return 0_V;

  // Convert motor RPS setpoint → mechanism rad/s
  auto setpointRad = units::radians_per_second_t{
      m_velocitySetpoint.value() * 2.0 * std::numbers::pi / kGearRatio};
  auto currentRad = m_feedSim.GetAngularVelocity();

  auto fb =
      units::volt_t{m_pid.Calculate(currentRad.value(), setpointRad.value())};

  auto total = fb;
  return units::math::max(units::math::min(total, 12_V), -12_V);
}

void SimIndexerIO::UpdateInputs(IndexerIOInputs &inputs) {
  constexpr auto dt = 20_ms;

  // Compute voltage based on mode
  units::volt_t feedVoltage{0_V};
  units::volt_t centerVoltage{0_V};

  switch (m_mode) {
  case Mode::kVoltage:
    feedVoltage = m_voltageCommand;
    centerVoltage = m_voltageCommand;
    break;
  case Mode::kVelocity:
    feedVoltage = CalculateClosedLoop();
    centerVoltage = feedVoltage; // center mirrors feed for simplicity
    break;
  case Mode::kStopped:
    break;
  }

  m_feedSim.SetInputVoltage(feedVoltage);
  m_centerSim.SetInputVoltage(centerVoltage);

  m_feedSim.Update(dt);
  m_centerSim.Update(dt);

  if (m_ballPresent) {
    m_ballTimer += dt;
    if (m_ballTimer < kBallContactDuration) {
      // Reduce velocity to simulate load (ball absorbs energy)
      m_feedSim.SetState(m_feedSim.GetAngularPosition(),
                         m_feedSim.GetAngularVelocity() * kBallLoadFactor);
    } else {
      m_ballPresent = false;
    }
  }

  inputs.motorPosition = units::turn_t{m_feedSim.GetAngularPosition().value() *
                                       kGearRatio / (2.0 * std::numbers::pi)};
  inputs.motorVelocity =
      units::turns_per_second_t{m_feedSim.GetAngularVelocity().value() *
                                kGearRatio / (2.0 * std::numbers::pi)};

  inputs.centerPosition =
      units::turn_t{m_centerSim.GetAngularPosition().value() * kGearRatio /
                    (2.0 * std::numbers::pi)};
  inputs.centerVelocity =
      units::turns_per_second_t{m_centerSim.GetAngularVelocity().value() *
                                kGearRatio / (2.0 * std::numbers::pi)};

  inputs.appliedVolts = feedVoltage;

  // GetCurrentDraw returns motor-level current
  inputs.statorCurrent = m_feedSim.GetCurrentDraw();
  inputs.centerStatorCurrent = m_centerSim.GetCurrentDraw();
  inputs.supplyCurrent = m_feedSim.GetCurrentDraw();

  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimIndexerIO::SetVoltage(units::volt_t voltage) {
  m_mode = Mode::kVoltage;
  m_voltageCommand = voltage;
}

void SimIndexerIO::SetMotorVelocity(units::turns_per_second_t rps) {
  m_mode = Mode::kVelocity;
  m_velocitySetpoint = rps;
}

void SimIndexerIO::Stop() {
  m_mode = Mode::kStopped;
  m_voltageCommand = 0_V;
  m_velocitySetpoint = units::turns_per_second_t{0};
}
