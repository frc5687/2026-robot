// Team 5687 2026

#include "subsystem/shooter/ShooterSystem.h"

#include <frc/Timer.h>

#include "frc/DriverStation.h"

ShooterSystem::ShooterSystem(
                             FlywheelSubsystem &flywheel, HoodSubsystem &hood)
    : CoordinatedSystem("Shooter"), m_flywheel(flywheel),
      m_hood(hood) {}

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
    auto setpoint = ShooterSetpoint{
        .turretAngle = solution.turretRobotAngle,
        .hoodAngle = solution.hoodAngle,
    };

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
}
