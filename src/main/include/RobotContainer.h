// Team 5687 2026

// RobotContainer.h
#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS5Controller.h>
#include <units/angle.h>

#include <memory>

#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/floorroller/FloorRollerSubsystem.h"
#include "subsystem/flywheel/FlywheelSubsystem.h"
#include "subsystem/hood/HoodSubsystem.h"
#include "subsystem/intake/IntakeSystem.h"
#include "subsystem/intake/bottomroller/IntakeBottomRollerSubsystem.h"
#include "subsystem/intake/deployer/IntakeDeployerSubsystem.h"
#include "subsystem/intake/toproller/IntakeTopRollerSubsystem.h"
#include "subsystem/kicker/KickerSubsystem.h"
#include "subsystem/shooter/ShooterSystem.h"
#include "subsystem/vision/VisionSubsystem.h"
#include "utils/TunableDouble.h"
#include "viz/RobotViz.h"

class RobotContainer {
public:
  RobotContainer();
  frc2::CommandPtr GetAutonomousCommand();
  void Periodic();

private:
  void ConfigureBindings();
  void ConfigureAutoCommands();

  std::unique_ptr<DriveSubsystem> CreateDrive();
  std::unique_ptr<FlywheelSubsystem> CreateFlywheel();
  std::unique_ptr<HoodSubsystem> CreateHood();
  std::unique_ptr<VisionSubsystem> CreateVision();
  std::unique_ptr<FloorRollerSubsystem> CreateFloorRoller();
  std::unique_ptr<KickerSubsystem> CreateKicker();
  std::unique_ptr<IntakeDeployerSubsystem> CreateIntakeDeployer();
  std::unique_ptr<IntakeTopRollerSubsystem> CreateIntakeTopRoller();
  std::unique_ptr<IntakeBottomRollerSubsystem> CreateIntakeBottomRoller();

  std::unique_ptr<DriveSubsystem> m_drive;
  std::unique_ptr<FlywheelSubsystem> m_flywheel;
  std::unique_ptr<HoodSubsystem> m_hood;
  std::unique_ptr<FloorRollerSubsystem> m_floorRoller;
  std::unique_ptr<KickerSubsystem> m_kicker;
  std::unique_ptr<IntakeDeployerSubsystem> m_intakeDeployer;
  std::unique_ptr<IntakeTopRollerSubsystem> m_intakeTopRoller;
  std::unique_ptr<IntakeBottomRollerSubsystem> m_intakeBottomRoller;
  std::unique_ptr<IntakeSystem> m_intake;
  std::unique_ptr<VisionSubsystem> m_vision;

  std::unique_ptr<ShooterSystem> m_shooter;

  TunableDouble m_simpleShootFlywheelRPM{"SmartDashboard",
                                         "SimpleShoot/FlywheelRPM", 1600.0};
  TunableDouble m_simpleShootKickerRPS{"SmartDashboard",
                                       "SimpleShoot/KickerRPS", 60.0};
  TunableDouble m_simpleShootAngle{"SmartDashboard", "SimpleShoot/AngleDeg",
                                   10.0};

  RobotViz m_robotViz{};
  frc2::CommandPS5Controller m_driver{0};
};
