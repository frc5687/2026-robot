// Team 5687 2026

#include "subsystem/shooter/ShooterSystem.h"

#include <frc/Timer.h>

#include "frc/DriverStation.h"

ShooterSystem::ShooterSystem(FlywheelSubsystem &flywheel, HoodSubsystem &hood)
    : CoordinatedSystem("Shooter"), m_flywheel(flywheel), m_hood(hood) {}

void ShooterSystem::SetState(const ShooterState &state) {
  m_previousState = m_currentState;
  m_currentState = state;
}

void ShooterSystem::SetSetpoint(const ShooterSetpoint &setpoint) {
  m_desiredSetpoint = setpoint;
}

void ShooterSystem::Update() {
  switch (m_currentState) {
  case ShooterState::IDLE:
    break;
  case ShooterState::TRACKING: {
    auto alliance = frc::DriverStation::GetAlliance();
    auto solution = m_shotCalculator.Calculate(
        frc::Timer::GetFPGATimestamp(), alliance == frc::DriverStation::kRed);
    ShooterSetpoint setpoint;
    setpoint.driveAngle = solution.driveAngle;
    setpoint.hoodAngle = solution.hoodAngle;
    setpoint.flywheelSpeed = units::revolutions_per_minute_t{solution.flywheelSpeed};

    SetSetpoint(setpoint);
  } break;
  case ShooterState::PASSING:
    break;
  case ShooterState::SHOOTING:
    break;
  default:
    break;
  }
}

void ShooterSystem::LogTelemetry() {
    // THese are to avoid warning for not used atm
    Log("Flywheel At Setpoint", m_flywheel.AtSetpoint());
    Log("Hood Angle", m_hood.GetPosition().value());
}
