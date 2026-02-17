#include "subsystem/shooter/ShooterSystem.h"
#include "frc/DriverStation.h"
#include <frc/Timer.h>

ShooterSystem::ShooterSystem(TurretSubsystem &turret,
                             FlywheelSubsystem &flywheel,
                             HoodSubsystem &hood)
: CoordinatedSystem("Shooter"), m_turret(turret), m_flywheel(flywheel), m_hood(hood) {}

void ShooterSystem::SetState(const ShooterState &state) {
    m_previousState = m_currentState;
    m_currentState = state;
}

void ShooterSystem::SetSetpoint(const ShooterSetpoint &setpoint) {
    m_desiredSetpoint = setpoint;
    m_turret.SetAngle(m_desiredSetpoint.turretAngle);
}

void ShooterSystem::Update() {
    switch (m_currentState) {
        case ShooterState::IDLE:
            break;
        case ShooterState::TRACKING: {
            auto alliance = frc::DriverStation::GetAlliance();
            auto solution = m_shotCalculator.Calculate(frc::Timer::GetFPGATimestamp(),
                                       alliance == frc::DriverStation::kRed);
            auto setpoint = ShooterSetpoint{
                .turretAngle=solution.turretRobotAngle,
                .hoodAngle=solution.hoodAngle,
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
    Log("Current State", ShooterStateToString(m_currentState));
    Log("Previous State", ShooterStateToString(m_previousState));
}
