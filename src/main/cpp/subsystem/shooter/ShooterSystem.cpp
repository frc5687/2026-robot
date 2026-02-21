// Team 5687 2026
#include "subsystem/shooter/ShooterSystem.h"

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>

#include "RobotState.h"
#include "frc/DriverStation.h"
#include "units/angular_velocity.h"

ShooterSystem::ShooterSystem(TurretSubsystem &turret,
                             FlywheelSubsystem &flywheel, HoodSubsystem &hood)
    : CoordinatedSystem("Shooter"),
      m_turret(turret),
      m_flywheel(flywheel),
      m_hood(hood) {}

void ShooterSystem::SetState(const ShooterState &state) {
  m_previousState = m_currentState;
  m_currentState = state;
}

void ShooterSystem::SetSetpoint(const ShooterSetpoint &setpoint) {
  m_desiredSetpoint = setpoint;
  m_hood.SetHoodPosition(units::turn_t{setpoint.hoodAngle}, units::turn_t{setpoint.hoodAngle});
  m_flywheel.SetRPM(setpoint.flywheelSpeed, setpoint.flywheelSpeed);
  m_turret.SetAngle(m_desiredSetpoint.turretAngle);
}

void ShooterSystem::Update() {
  switch (m_currentState) {
    case ShooterState::IDLE:
      break;

    case ShooterState::TRACKING: {
      auto alliance = frc::DriverStation::GetAlliance();
      bool isRed = alliance == frc::DriverStation::kRed;

      auto solution = m_shotCalculator.Calculate(
          frc::Timer::GetFPGATimestamp(), isRed);

      Log("Solution/turretFieldAngle",
          solution.turretFieldAngle.Radians().value());
      Log("Solution/turretRobotAngle", solution.turretRobotAngle.value());
      Log("Solution/leftHoodAngle", solution.leftHoodAngle.value());
      Log("Solution/rightHoodAngle", solution.rightHoodAngle.value());
      Log("Solution/leftFlywheelSpeed", solution.leftFlywheelSpeed);
      Log("Solution/rightFlywheelSpeed", solution.rightFlywheelSpeed);
      Log("Solution/leftDistance", solution.leftDistance.value());
      Log("Solution/rightDistance", solution.rightDistance.value());
      Log("Solution/angularDivergence", solution.angularDivergence.value());
      Log("Solution/inRange", solution.inRange);
      Log("Solution/ready", solution.ready);
      Log("Solution/divergenceOK", solution.divergenceOK);

      auto cfg = m_shotCalculator.GetConfig();

      frc::Translation2d goal = isRed
          ? frc::Translation2d{
                units::meter_t{Constants::Field::kFieldLength} -
                    cfg.targetXY.X(),
                cfg.targetXY.Y()}
          : cfg.targetXY;
      Log("Viz/Goal", frc::Pose2d{goal, frc::Rotation2d{0_rad}});

      auto &rs = RobotState::Instance();
      frc::Pose2d robotPose =
          rs.GetDriveState(frc::Timer::GetFPGATimestamp()).estimatedPose;

      frc::Pose2d aimPose{robotPose.Translation(),
                          solution.turretFieldAngle};
      Log("Viz/AimPose", aimPose);

      double maxDist = std::max(solution.leftDistance.value(),
                                solution.rightDistance.value());
      frc::Translation2d projectedHit{
          robotPose.X() +
              units::meter_t{maxDist * solution.turretFieldAngle.Cos()},
          robotPose.Y() +
              units::meter_t{maxDist * solution.turretFieldAngle.Sin()}};
      Log("Viz/ProjectedHit",
          frc::Pose2d{projectedHit, solution.turretFieldAngle});

      frc::Pose2d leftPose =
          robotPose.TransformBy(cfg.robotToLeftLauncher);
      frc::Pose2d rightPose =
          robotPose.TransformBy(cfg.robotToRightLauncher);
      Log("Viz/LeftLauncher",
          frc::Pose2d{leftPose.Translation(), solution.turretFieldAngle});
      Log("Viz/RightLauncher",
          frc::Pose2d{rightPose.Translation(), solution.turretFieldAngle});

      double aimError = projectedHit.Distance(goal).value();
      Log("Viz/AimErrorMeters", aimError);

      auto setpoint = ShooterSetpoint{
          .turretAngle = solution.turretRobotAngle,
          .hoodAngle = solution.leftHoodAngle,
          .flywheelSpeed = units::revolutions_per_minute_t{solution.leftFlywheelSpeed},
      };
      SetSetpoint(setpoint);
    } break;

    case ShooterState::PASSING:
      break;

    case ShooterState::SHOOTING:
      break;

    case ShooterState::TRENCH: {
      auto setpoint = ShooterSetpoint{.hoodAngle = 0_tr, .flywheelSpeed = 0_rpm};
      SetSetpoint(setpoint);
    }
    default:
      break;
  }
}

void ShooterSystem::LogTelemetry() {
  Log("Current State", ShooterStateToString(m_currentState));
  Log("Previous State", ShooterStateToString(m_previousState));
}
