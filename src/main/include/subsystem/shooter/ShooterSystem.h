// Team 5687 2026

#pragma once

#include <units/angle.h>
#include <units/velocity.h>

#include "ShotCalculator.h"
#include "subsystem/Turret/TurretSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/shooter/CoordinatedSystem.h"
#include "units/angular_velocity.h"

enum class ShooterState {
  IDLE,
  TRACKING,
  PASSING,
  SHOOTING,
  TRENCH,
};

inline constexpr std::string ShooterStateToString(ShooterState state) {
  switch (state) {
  case ShooterState::IDLE:
    return "IDLE";
  case ShooterState::TRACKING:
    return "TRACKING";
  case ShooterState::PASSING:
    return "PASSING";
  case ShooterState::SHOOTING:
    return "SHOOTING";
  case ShooterState::TRENCH:
    return "TRENCH";
  default:
    return "UNKNOWN";
  }
}

struct ShooterSetpoint {
  units::radian_t turretAngle;
  units::turn_t hoodAngle;
  units::revolutions_per_minute_t flywheelSpeed;
};

// Combination of Turret, Shooter, Hood.
// Turret is a bit strange since both are coupled together.
class ShooterSystem : public CoordinatedSystem {
public:
  ShooterSystem(TurretSubsystem &turret, FlywheelSubsystem &flywheel,
                HoodSubsystem &hood);
  void Update() override;
  void SetSetpoint(const ShooterSetpoint &setpoint);
  void SetState(const ShooterState &state);

protected:
  void LogTelemetry() override;

private:
  TurretSubsystem &m_turret;
  FlywheelSubsystem &m_flywheel;
  HoodSubsystem &m_hood;

  ShooterState m_currentState{ShooterState::IDLE};
  ShooterState m_previousState{ShooterState::IDLE};

  ShooterSetpoint m_desiredSetpoint;
  // LaunchConfig m_config{
  //     .robotToLauncher{Constants::Geometry::kRobotToTurretLeft.Translation().ToTranslation2d()},
  // };
  ShotCalculator m_shotCalculator;
};
